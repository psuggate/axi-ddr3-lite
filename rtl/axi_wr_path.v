`timescale 1ns / 100ps
/**
 * Write datapath, for an AXI4 to SDRAM interface.
 *
 * For every WRITE-request, store one write command for each BL8 transaction
 * that is required to complete an AXI4 (burst-) WRITE request. The WRITE-data
 * FIFO stores all (burst-)data as one write-data packet.
 *
 * Note: WRITE-data buffering is used, so the AXI4 interface may accept multiple
 * packets before any are actually written to the SDRAM.
 *
 * Copyright 2023, Patrick Suggate.
 *
 */
module axi_wr_path #(
    parameter  ADDRS = 32,
    localparam ASB   = ADDRS - 1,

    parameter  WIDTH = 32,
    localparam MSB   = WIDTH - 1,

    parameter  MASKS = WIDTH / 8,
    localparam SSB   = MASKS - 1,

    parameter AXI_ID_WIDTH = 4,
    localparam ISB = AXI_ID_WIDTH - 1,

    parameter CTRL_FIFO_DEPTH = 16,
    parameter CTRL_FIFO_BLOCK = 0,
    localparam CBITS = $clog2(CTRL_FIFO_DEPTH),

    parameter DATA_FIFO_BYPASS = 0,
    parameter DATA_FIFO_DEPTH = 512,
    parameter DATA_FIFO_BLOCK = 1,
    localparam DBITS = $clog2(DATA_FIFO_DEPTH),

    parameter USE_PACKET_FIFOS = 1,
    parameter USE_SYNC_FIFO = 0
) (
    input clock,
    input reset,

    input axi_awvalid_i,  // AXI4 Write Address Port
    output axi_awready_o,
    input [ASB:0] axi_awaddr_i,
    input [ISB:0] axi_awid_i,
    input [7:0] axi_awlen_i,
    input [1:0] axi_awburst_i,
    input axi_wvalid_i,  // AXI4 Write Data Port
    output axi_wready_o,
    input [MSB:0] axi_wdata_i,
    input [SSB:0] axi_wstrb_i,
    input axi_wlast_i,
    output axi_bvalid_o,  // AXI4 Write Response
    input axi_bready_i,
    output [1:0] axi_bresp_o,
    output [ISB:0] axi_bid_o,

    output mem_store_o,
    input mem_accept_i,
    output mem_wseq_o,
    output [ISB:0] mem_wrid_o,
    output [ASB:0] mem_addr_o,

    output mem_valid_o,
    input mem_ready_i,
    output mem_last_o,
    output [SSB:0] mem_strb_o,
    output [MSB:0] mem_data_o
);

  //
  // Todo:
  //  - padding with empty-words for unaligned and/or small transfers
  //  - any advantage to accepting commands _before_ data ??
  //

  // -- Constants -- //

  localparam [1:0] BURST_INCR = 2'b01;
  localparam [1:0] AXI_RESP_OKAY = 2'b00;

  // Bit-width of the command-info that is stored in the WR-command FIFO
  localparam COMMAND_WIDTH = ADDRS + AXI_ID_WIDTH + 1;
  localparam WSB = COMMAND_WIDTH - 1;

  // States for capturing write requests
  localparam ST_IDLE = 1;
  localparam ST_FILL = 2;
  localparam ST_BUSY = 4;

  reg bvalid, aready, wready;
  reg [  2:0] state;
  reg [  1:0] bresp;
  reg [ISB:0] bwrid;

  wire wdf_ready, wdf_valid, wdf_last, dat_ready;
  wire cmd_ready, cmd_valid, wcf_valid;
  wire [WSB:0] command_w;
  wire [SSB:0] wdf_strb;
  wire [MSB:0] wdf_data;

  wire xvalid, xready, xseq;
  wire [ISB:0] xid;
  wire [ASB:0] xaddr;

  assign axi_awready_o = aready;
  assign axi_wready_o = DATA_FIFO_BYPASS ? mem_ready_i : wready;
  assign axi_bvalid_o = bvalid;
  assign axi_bresp_o = bresp;
  assign axi_bid_o = bwrid;

  assign mem_store_o = wcf_valid;
  assign mem_valid_o = DATA_FIFO_BYPASS ? axi_wvalid_i : wdf_valid;
  assign mem_last_o = DATA_FIFO_BYPASS ? axi_wlast_i : wdf_last;
  assign mem_strb_o = DATA_FIFO_BYPASS ? axi_wstrb_i : wdf_strb;
  assign mem_data_o = DATA_FIFO_BYPASS ? axi_wdata_i : wdf_data;

  assign dat_ready = DATA_FIFO_BYPASS ? 1'b1 : wdf_ready;

  // Command- & data- FIFO signals
  assign cmd_valid = axi_awvalid_i & aready;
  assign command_w = {axi_awaddr_i, axi_awid_i};


  // -- FSM to Capture WRITE Requests and Data -- //

  always @(posedge clock) begin
    if (reset) begin
      state  <= ST_IDLE;
      aready <= 1'b0;
      wready <= 1'b0;
    end else begin
      case (state)
        ST_IDLE: begin
          // Wait for incoming write-data requests
          if (cmd_valid) begin
            state  <= ST_FILL;
            aready <= 1'b0;
            wready <= 1'b1;
          end else begin
            state  <= ST_IDLE;
            aready <= 1'b1;
            wready <= 1'b0;
          end
        end

        ST_FILL: begin
          aready <= 1'b0;

          // Wait for the write-data to be stored
          if (axi_wvalid_i && wready && axi_wlast_i) begin
            wready <= 1'b0;
            if (!cmd_ready || !dat_ready) begin
              state <= ST_BUSY;
            end else begin
              state <= ST_IDLE;
            end
          end else begin
            state  <= ST_FILL;
            wready <= 1'b1;
          end
        end

        ST_BUSY: begin
          // If either FIFO fills up, then wait for a bit
          if (cmd_ready && dat_ready) begin
            state  <= ST_IDLE;
            aready <= 1'b1;
          end else begin
            state  <= ST_BUSY;
            aready <= 1'b0;
          end
          wready <= 1'b0;
        end

        default: begin
          $error("%10t: WRITE data state-machine failure!", $time);
          state  <= ST_IDLE;
          aready <= 1'b0;
          wready <= 1'b0;
          // $fatal;
        end
      endcase  // state
    end
  end


  // -- AXI WRITE-Response Logic -- //

  always @(posedge clock) begin
    if (reset) begin
      bvalid <= 1'b0;
      bresp  <= AXI_RESP_OKAY;
      bwrid  <= 'bx;
    end else begin
      bresp <= AXI_RESP_OKAY;

      if (mem_accept_i && !mem_wseq_o) begin
        bwrid <= mem_wrid_o;
      end else begin
        bwrid <= bwrid;
      end

      if (mem_ready_i && mem_valid_o && mem_last_o) begin
        bvalid <= 1'b1;
      end else if (bvalid && axi_bready_i) begin
        bvalid <= 1'b0;
      end
    end
  end


  // -- Chunker for Large Bursts -- //

  axi_chunks #(
      .ADDRS(ADDRS),
      .REQID(AXI_ID_WIDTH)
  ) U_CHUNK1 (
      .clock(clock),
      .reset(reset),

      .avalid_i(axi_awvalid_i & aready),
      .aready_o(cmd_ready),
      .alen_i(axi_awlen_i),
      .aburst_i(axi_awburst_i),
      .aid_i(axi_awid_i),
      .aaddr_i(axi_awaddr_i),

      .xvalid_o(xvalid),
      .xready_i(xready),
      .xseq_o(xseq),
      .xid_o(xid),
      .xaddr_o(xaddr)
  );


  // -- Write-Data Command FIFO -- //

  sync_fifo #(
      .WIDTH (COMMAND_WIDTH),
      .ABITS (CBITS),
      .OUTREG(CTRL_FIFO_BLOCK)
  ) U_FIFO1 (
      .clock  (clock),
      .reset  (reset),
      .level_o(),

      .valid_i(xvalid),
      .ready_o(xready),
      .data_i ({xaddr, xid, xseq}),

      .valid_o(wcf_valid),
      .ready_i(mem_accept_i),
      .data_o ({mem_addr_o, mem_wrid_o, mem_wseq_o})
  );


  // -- Synchronous, 2 kB, Write-Data FIFO -- //

  //
  // Todo:
  //  - not required if the data flows directly to a 'Bulk EP IN' core, as these
  //    already contain sufficient buffering ??
  //  - use a packet FIFO, so we only bother the AXI bus when we have a full
  //    frame of data to transfer ??
  //  - pad end of bursts ??
  //
  generate
    if (USE_SYNC_FIFO && USE_PACKET_FIFOS) begin : g_sync_fifo

      packet_fifo #(
          .WIDTH(MASKS + WIDTH),
          .DEPTH(1 << DBITS),
          .SAVE_ON_LAST(1),
          .LAST_ON_SAVE(0),
          .NEXT_ON_LAST(1),
          .OUTREG(DATA_FIFO_BLOCK)
      ) U_FIFO2 (
          .clock(clock),
          .reset(reset),

          .level_o(),
          .drop_i (1'b0),
          .save_i (1'b0),
          .redo_i (1'b0),
          .next_i (1'b0),

          .s_tvalid(axi_wvalid_i & wready),
          .s_tready(wdf_ready),
          .s_tkeep (1'b1),
          .s_tlast (axi_wlast_i),
          .s_tdata ({axi_wstrb_i, axi_wdata_i}), // todo: pad end of bursts ??

          .m_tvalid(wdf_valid),
          .m_tready(mem_ready_i),
          .m_tlast (wdf_last),
          .m_tdata ({wdf_strb, wdf_data})
      );

    end else begin : g_axis_fifo

      wire s_tvalid = axi_wvalid_i & wready;

      axis_fifo #(
          .DEPTH(DATA_FIFO_DEPTH),
          .DATA_WIDTH(WIDTH),
          .KEEP_ENABLE(1),
          .KEEP_WIDTH(MASKS),
          .LAST_ENABLE(1),
          .ID_ENABLE(0),
          .ID_WIDTH(1),
          .DEST_ENABLE(0),
          .DEST_WIDTH(1),
          .USER_ENABLE(0),
          .USER_WIDTH(1),
          .RAM_PIPELINE(DATA_FIFO_BLOCK),
          .OUTPUT_FIFO_ENABLE(0),
          .FRAME_FIFO(USE_PACKET_FIFOS),
          .USER_BAD_FRAME_VALUE(0),
          .USER_BAD_FRAME_MASK(0),
          .DROP_BAD_FRAME(0),
          .DROP_WHEN_FULL(0)
      ) U_FIFO4 (
          .clk(clock),
          .rst(reset),

          .s_axis_tvalid(s_tvalid),
          .s_axis_tready(wdf_ready),
          .s_axis_tkeep(axi_wstrb_i),
          .s_axis_tlast(axi_wlast_i),
          .s_axis_tid(1'b0),
          .s_axis_tdest(1'b0),
          .s_axis_tuser(1'b0),
          .s_axis_tdata(axi_wdata_i),

          .pause_req(1'b0),
          .pause_ack(),

          .m_axis_tvalid(wdf_valid),
          .m_axis_tready(mem_ready_i),
          .m_axis_tkeep(wdf_strb),
          .m_axis_tlast(wdf_last),
          .m_axis_tid(),
          .m_axis_tdest(),
          .m_axis_tuser(),
          .m_axis_tdata(wdf_data),

          .status_depth(),
          .status_depth_commit(),
          .status_overflow(),
          .status_bad_frame(),
          .status_good_frame()
      );

    end
  endgenerate  /* !SYNC_FIFO */

`ifdef __icarus
  //
  //  Simulation Only
  ///

  reg [39:0] dbg_state;

  always @* begin
    case (state)
      ST_IDLE: dbg_state = "IDLE";
      ST_FILL: dbg_state = "FILL";
      ST_BUSY: dbg_state = "BUSY";
      default: dbg_state = " ?? ";
    endcase
  end

  always @(posedge clock) begin
    if (reset);
    else begin
      if (axi_awvalid_i && axi_awburst_i != BURST_INCR) begin
        $error("%10t: Only 'INCR' WRITE bursts are supported", $time);
        // $fatal;
      end

      // todo: temporary restrictions ...
      if (axi_awvalid_i && axi_awlen_i[1:0] != 2'd3) begin
        $error("%10t: Only WRITE bursts that are multiples of 16-bytes are supported", $time);
        // $fatal;
      end
      if (axi_awvalid_i && axi_awaddr_i[2:0] != 3'd0) begin
        $error("%10t: Only 16-byte-aligned WRITE bursts are supported", $time);
        // $fatal;
      end
      // odot: temporary restrictions ...
    end
  end

`endif  /* !__icarus */


endmodule  /* axi_wr_path */
