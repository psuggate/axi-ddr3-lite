`timescale 1ns / 100ps
/**
 * Write datapath, for an AXI4 to SDRAM interface.
 * 
 * For every write-request, store one write command, and one write-data packet.
 * Write-data buffering is used, so the AXI4 interface may accept multiple
 * packets before any are actually written to the SDRAM.
 * 
 * Copyright 2023, Patrick Suggate.
 * 
 */
module axi_wr_path (
    clock,
    reset,

    axi_awvalid_i,
    axi_awready_o,
    axi_awaddr_i,
    axi_awid_i,
    axi_awlen_i,
    axi_awburst_i,

    axi_wvalid_i,
    axi_wready_o,
    axi_wlast_i,
    axi_wstrb_i,
    axi_wdata_i,

    axi_bvalid_o,
    axi_bready_i,
    axi_bresp_o,
    axi_bid_o,

    mem_store_o,
    mem_accept_i,
    mem_wrid_o,
    mem_addr_o,

    mem_valid_o,
    mem_ready_i,
    mem_last_o,
    mem_strb_o,
    mem_data_o
);


  // todo: Issue an AXI4 write-response once the write-data has been buffered,
  //   but before the memory-controller has performed the write? This could
  //   cause read-after-write problems, depending on the order that the
  //   transactions are actually performed.
  parameter EAGER_RESPONSE = 0;

  parameter ADDRS = 32;
  localparam ASB = ADDRS - 1;

  parameter WIDTH = 32;
  localparam MSB = WIDTH - 1;

  parameter MASKS = WIDTH / 8;
  localparam SSB = MASKS - 1;

  parameter AXI_ID_WIDTH = 4;
  localparam ISB = AXI_ID_WIDTH - 1;

  parameter CTRL_FIFO_DEPTH = 16;
  parameter CTRL_FIFO_BLOCK = 0;
  localparam CBITS = $clog2(CTRL_FIFO_DEPTH);

  parameter DATA_FIFO_DEPTH = 512;
  parameter DATA_FIFO_BLOCK = 1;
  localparam DBITS = $clog2(DATA_FIFO_DEPTH);


  input clock;
  input reset;

  input axi_awvalid_i;  // AXI4 Write Address Port
  output axi_awready_o;
  input [ASB:0] axi_awaddr_i;
  input [ISB:0] axi_awid_i;
  input [7:0] axi_awlen_i;
  input [1:0] axi_awburst_i;
  input axi_wvalid_i;  // AXI4 Write Data Port
  output axi_wready_o;
  input [MSB:0] axi_wdata_i;
  input [SSB:0] axi_wstrb_i;
  input axi_wlast_i;
  output axi_bvalid_o;  // AXI4 Write Response
  input axi_bready_i;
  output [1:0] axi_bresp_o;
  output [ISB:0] axi_bid_o;

  output mem_store_o;  // todo: good ??
  input mem_accept_i;  // todo: good ??
  output [ISB:0] mem_wrid_o;
  output [ASB:0] mem_addr_o;

  output mem_valid_o;
  input mem_ready_i;
  output mem_last_o;
  output [SSB:0] mem_strb_o;
  output [MSB:0] mem_data_o;


  // -- Constants -- //

  localparam [1:0] BURST_INCR = 2'b01;
  localparam [1:0] AXI_RESP_OKAY = 2'b00;


`ifdef __icarus
  always @(posedge clock) begin
    if (reset);
    else begin
      if (axi_awvalid_i && axi_awburst_i != BURST_INCR) begin
        $error("%10t: Only 'INCR' WRITE bursts are supported", $time);
        $fatal;
      end

      // todo: temporary restrictions ...
      if (axi_awvalid_i && axi_awlen_i != 8'd3) begin
        $error("%10t: Only 16-byte sized WRITE bursts are supported", $time);
        $fatal;
      end
      if (axi_awvalid_i && axi_awaddr_i[6:0] != 7'd0) begin
        $error("%10t: Only 16-byte-aligned WRITE bursts are supported", $time);
        $fatal;
      end
      // odot: temporary restrictions ...
    end
  end
`endif


  // todo:
  //  - burst counter, so that responses can be sent on all RX data
  //  - padding with empty-words for unaligned and/or small transfers
  //  - FSM that only accepts write-data from a single source ??
  //  - any advantage to accepting commands _before_ data ??


  reg wlast, mlast, mvalid, bvalid, write, aready, wready;
  reg [  1:0] bresp;
  reg [ISB:0] bwrid;

  wire cmd_ready, wdf_ready, wdf_valid, wdf_last, wr_accept;
  wire wcf_valid;


  assign axi_awready_o = aready;
  assign axi_wready_o = wready;
  assign axi_bvalid_o = bvalid;
  assign axi_bresp_o = bresp;
  assign axi_bid_o = bwrid;

  assign mem_store_o = write;
  assign mem_valid_o = mvalid;
  assign mem_last_o = mlast;

  assign wr_accept = write & mem_accept_i;


  // -- Chunk-up Large Write-Data Bursts -- //

  // note: chunks are good, because of row/col/bank/page boundaries, of SDRAMs
  reg [7:0] transfers_remaining;

  always @(posedge clock) begin
    if (reset) begin
      transfers_remaining <= 'bx;  // toods
    end else begin
      transfers_remaining <= axi_awlen_i;
    end
  end

  reg [1:0] fill_count, xfer_count;
  reg [3:0] state, issue;

  wire [1:0] fill_cnext = fill_count - 1;
  wire [1:0] xfer_cnext = xfer_count - 1;


  // -- FSM to Capture WRITE Requests and Data -- //

  // States for capturing write requests
  localparam ST_IDLE = 4'b0000;
  localparam ST_FILL = 4'b0001;
  localparam ST_BUSY = 4'b0010;

  always @(posedge clock) begin
    if (reset) begin
      state  <= ST_IDLE;
      aready <= 1'b0;
      wready <= 1'b0;
      wlast  <= 1'b0;
    end else begin
      case (state)
        ST_IDLE: begin
          // Wait for incoming write-data requests
          if (axi_awvalid_i && aready) begin
            state <= ST_FILL;
            fill_count <= 2'd3;
            aready <= 1'b0;
            wready <= 1'b1;
          end else begin
            fill_count <= 2'bxx;
            aready <= 1'b1;
            wready <= 1'b0;
          end
        end

        ST_FILL: begin
          aready <= 1'b0;

          // Wait for the write-data to be stored
          if (axi_wvalid_i && wready && fill_count == 2'd0) begin
            // if (axi_wvalid_i && wready && wlast) begin // todo: faster ??
            wready <= 1'b0;
            if (!cmd_ready || !wdf_ready) begin
              state <= ST_BUSY;
            end else begin
              state <= ST_IDLE;
            end
          end else if (axi_wvalid_i && wready) begin
            fill_count <= fill_cnext;
            // wready <= 1'b1;
          end

          // Use the 'wlast' flag to update the packet-counter
          if (axi_wvalid_i && wready && fill_count == 2'd1) begin
            wlast <= 1'b1;
          end else if (axi_wvalid_i && wready) begin
            wlast <= 1'b0;
          end
        end

        ST_BUSY: begin
          // If either FIFO fills up, then wait for a bit
          if (cmd_ready && wdf_ready) begin
            state  <= ST_IDLE;
            aready <= 1'b1;
          end else begin
            aready <= 1'b0;
          end
        end

        default: begin
          $error("%10t: WRITE data state-machine failure!", $time);
          $fatal;
        end
      endcase  // state
    end
  end


  // -- FSM to Send WRITE Requests & Data to the Memory Controller -- //

  // States for issuing write commands
  localparam IS_IDLE = 4'b0000;
  localparam IS_XFER = 4'b0001;
  localparam IS_RESP = 4'b0010;

  always @(posedge clock) begin
    if (reset) begin
      issue  <= IS_IDLE;
      bvalid <= 1'b0;
      bresp  <= 2'bxx;
      mvalid <= 1'b0;
      mlast  <= 1'b0;
    end else begin

      case (issue)
        IS_IDLE: begin
          // Wait for the memory-controller to accept the command
          if (write && mem_accept_i) begin
            xfer_count <= 2'd3;
            issue <= IS_XFER;
            bwrid <= mem_wrid_o;  // toods ...
            mvalid <= 1'b1;
          end else begin
            xfer_count <= 2'bxx;
            bwrid <= {AXI_ID_WIDTH{1'bx}};
            mvalid <= 1'b0;
          end
          bvalid <= 1'b0;
          mlast  <= 1'b0;
        end

        IS_XFER: begin
          // Transfer all write-data to the memory controller
          if (wdf_valid && mem_ready_i && xfer_count == 2'd0) begin
            issue  <= IS_RESP;
            mvalid <= 1'b0;
            bvalid <= 1'b1;
            bresp  <= AXI_RESP_OKAY;
          end else begin
            mvalid <= 1'b1;
            bvalid <= 1'b0;

            if (wdf_valid && mem_ready_i) begin
              xfer_count <= xfer_cnext;
            end
          end

          if (wdf_valid && mem_ready_i && xfer_count == 2'd1) begin
            mlast <= 1'b1;
          end else if (wdf_valid && mem_ready_i) begin
            mlast <= 1'b0;
          end
        end

        IS_RESP: begin
          // Issue the AXI4 write response
          if (bvalid && axi_bready_i) begin
            issue  <= IS_IDLE;
            bvalid <= 1'b0;
            bresp  <= 2'bxx;
          end else begin
            bvalid <= 1'b1;
            bresp  <= AXI_RESP_OKAY;
          end
        end

        default: begin
          $error("%10t: WRITE issue state-machine failure!", $time);
          $fatal;
        end
      endcase  // issue
    end
  end


  // -- Issue Write-Data Requests -- //

  always @(posedge clock) begin
    if (reset) begin
      write <= 1'b0;
    end else begin
      if (issue == IS_XFER || write && mem_accept_i) begin
        // Memory controller has accepted the WRITE request, and (possibly) data-
        // transfer to the controller is already happening -- don't issue another
        // WRITE request until we are no longer transferring.
        write <= 1'b0;
      end else if (wcf_valid && axi_wvalid_i && wready && axi_wlast_i) begin
        // If there is a command ready, and all write-data has been transferred,
        // then issue a WRITE request to the memory controller
        write <= 1'b1;
      end
    end
  end


  // -- Write-Data Command FIFO -- //

  localparam COMMAND_WIDTH = ADDRS + AXI_ID_WIDTH;
  localparam WSB = COMMAND_WIDTH - 1;

  wire cmd_valid = axi_awvalid_i & aready;
  wire [WSB:0] command_w;

  assign command_w = {axi_awaddr_i, axi_awid_i};

  sync_fifo #(
      .WIDTH (COMMAND_WIDTH),
      .ABITS (CBITS),
      .OUTREG(CTRL_FIFO_BLOCK)
  ) command_fifo_inst (
      .clock(clock),
      .reset(reset),

      .valid_i(cmd_valid),
      .ready_o(cmd_ready),
      .data_i (command_w),

      .valid_o(wcf_valid),
      .ready_i(wr_accept),
      .data_o ({mem_addr_o, mem_wrid_o})
  );


  // -- Synchronous, 2 kB, Write-Data FIFO -- //

  packet_fifo #(
      .WIDTH (MASKS + WIDTH),
      .ABITS (DBITS),
      .OUTREG(DATA_FIFO_BLOCK)
  ) wrdata_fifo_inst (
      .clock(clock),
      .reset(reset),

      .valid_i(axi_wvalid_i),
      .ready_o(wdf_ready),
      .last_i (wlast),
      .drop_i (1'b0),
      .data_i ({axi_wstrb_i, axi_wdata_i}), // todo: pad end of bursts

      .valid_o(wdf_valid),
      .ready_i(mem_ready_i),
      .last_o (),
      .data_o ({mem_strb_o, mem_data_o})
  );


  // -- More Simulation Assertions -- //

`ifdef __icarus
  always @(posedge clock) begin
    if (reset);
    else begin
      if (axi_wvalid_i && axi_wready_o && axi_wlast_i != wlast) begin
        $error("%10t: 'wlast' signals disagree, for WRITE burst", $time);
        $fatal;
      end
    end
  end
`endif


endmodule  // axi_wr_path
