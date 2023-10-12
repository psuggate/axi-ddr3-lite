`timescale 1ns / 100ps
module axi_rd_path (
    clock,
    reset,

    axi_arvalid_i,
    axi_arready_o,
    axi_araddr_i,
    axi_arid_i,
    axi_arlen_i,
    axi_arburst_i,

    axi_rready_i,
    axi_rvalid_o,
    axi_rlast_o,
    axi_rresp_o,
    axi_rid_o,
    axi_rdata_o,

    mem_fetch_o,
    mem_accept_i,
    mem_rdid_o,
    mem_addr_o,
    mem_valid_i,
    mem_ready_o,
    mem_last_i,
    mem_rdid_i,
    mem_data_i
);

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

  input axi_arvalid_i;  // AXI4 Read Address Port
  output axi_arready_o;
  input [ISB:0] axi_arid_i;
  input [7:0] axi_arlen_i;
  input [1:0] axi_arburst_i;
  input [ASB:0] axi_araddr_i;

  input axi_rready_i;  // AXI4 Read Data Port
  output axi_rvalid_o;
  output [MSB:0] axi_rdata_o;
  output [1:0] axi_rresp_o;
  output [ISB:0] axi_rid_o;
  output axi_rlast_o;

  output mem_fetch_o;
  input mem_accept_i;
  output [ISB:0] mem_rdid_o;
  output [ASB:0] mem_addr_o;

  input mem_valid_i;
  output mem_ready_o;
  input mem_last_i;
  input [ISB:0] mem_rdid_i;
  input [MSB:0] mem_data_i;


  // -- Constants -- //

  localparam [1:0] BURST_INCR = 2'b01;
  localparam [1:0] AXI_RESP_OKAY = 2'b00;


`ifdef __icarus
  always @(posedge clock) begin
    if (reset);
    else begin
      if (axi_arvalid_i && axi_arburst_i != BURST_INCR) begin
        $error("%10t: Only 'INCR' READ bursts are supported", $time);
        $fatal;
      end
    end
  end
`endif


  reg aready, rready, fetch;

  wire cmd_valid = axi_arvalid_i & aready;
  wire cmd_ready, rcf_valid, rdf_ready, rrf_ready;


  assign axi_arready_o = aready;

  assign axi_rresp_o   = rrf_ready ? AXI_RESP_OKAY : 2'bxx;

  assign mem_fetch_o   = fetch;
  assign mem_ready_o   = rready;


  // -- Chunk-up Large Read-Data Bursts -- //

  reg [3:0] state;
  reg [1:0] chop_count;

  // States for capturing read requests
  localparam ST_IDLE = 4'b0000;
  localparam ST_CHOP = 4'b0001;
  localparam ST_READ = 4'b0010;
  localparam ST_BUSY = 4'b0100;

  always @(posedge clock) begin
    if (reset) begin
      state  <= ST_IDLE;
      aready <= 1'b0;
      rready <= 1'b0;
    end else begin
      case (state)
        ST_IDLE: begin
          // Wait for incoming read requests
          if (axi_arvalid_i && aready) begin
            state <= ST_CHOP;
            chop_count <= 2'd3;
            aready <= 1'b0;
          end else begin
            chop_count <= 2'bxx;
            aready <= 1'b1;
          end
        end

        ST_CHOP: begin
          // Break up large bursts into smaller bursts
          if (fetch && mem_accept_i) begin
            rready <= 1'b1;
            state  <= ST_READ;  // todoodo ...
          end else begin
            rready <= 1'b0;
          end
        end

        ST_READ: begin
          if (mem_valid_i && rready && mem_last_i) begin
            rready <= 1'b0;

            if (cmd_ready && rdf_ready) begin
              state <= ST_IDLE;
            end else begin
              state <= ST_BUSY;
            end
          end else begin
            rready <= rdf_ready;  // todo: skid-buf required ??
          end
        end

        ST_BUSY: begin
          if (cmd_ready && mem_ready_o) begin
            state  <= ST_IDLE;
            aready <= 1'b1;
          end else begin
            aready <= 1'b0;
          end
        end

        default: begin
          $error("RD:%10t: READ state machine failure!", $time);
          $fatal;
        end
      endcase
    end
  end


  // -- Issue Fetch-Data Requests -- //

  always @(posedge clock) begin
    if (reset) begin
      fetch <= 1'b0;
    end else begin
      if (state == ST_IDLE && cmd_ready && axi_arvalid_i && aready) begin
        fetch <= 1'b1;
      end else if (fetch && mem_accept_i) begin
        // Memory controller has accepted the FETCH request -- so do not issue
        // another FETCH request until 'n' transfers have started ...
        // todo: probably need some more control-signals for this flow-control
        //   logic ??
        fetch <= 1'b0;
      end
    end
  end


  // -- Read-Data Command FIFO -- //

  localparam COMMAND_WIDTH = ADDRS + AXI_ID_WIDTH;

  sync_fifo #(
      .WIDTH (COMMAND_WIDTH),
      .ABITS (CBITS),
      .OUTREG(CTRL_FIFO_BLOCK)
  ) command_fifo_inst (
      .clock(clock),
      .reset(reset),

      .valid_i(cmd_valid),
      .ready_o(cmd_ready),
      .data_i ({axi_araddr_i, axi_arid_i}),

      .valid_o(rcf_valid),
      .ready_i(fetch & mem_accept_i),
      .data_o ({mem_addr_o, mem_rdid_o})
  );


  // -- Read-Data Response FIFO -- //

  // todo: should this be in 'ddr3_axi_ctrl', so that it can be used for both
  //   READ & WRITE operations ??
  sync_fifo #(
      .WIDTH (AXI_ID_WIDTH),
      .ABITS (CBITS),
      .OUTREG(CTRL_FIFO_BLOCK)
  ) response_fifo_inst (
      .clock  (clock),
      .reset  (reset),
      .valid_i(fetch & mem_accept_i),
      .ready_o(),
      .data_i (mem_rdid_o),
      .valid_o(rrf_ready),
      .ready_i(axi_rvalid_o & axi_rready_i & axi_rlast_o),
      .data_o (axi_rid_o)
  );


  // -- Synchronous, 2 kB, Read-Data FIFO -- //

  // todo: use a packet FIFO, so we only bother the AXI bus when we have a full
  //   frame of data to transfer ??
  sync_fifo #(
      .WIDTH (WIDTH + 1),
      .ABITS (DBITS),
      .OUTREG(DATA_FIFO_BLOCK)
  ) rddata_fifo_inst (
      .clock(clock),
      .reset(reset),

      .valid_i(mem_valid_i),
      .ready_o(rdf_ready),
      .data_i ({mem_last_i, mem_data_i}), // todo: pad end of bursts ??

      .valid_o(axi_rvalid_o),
      .ready_i(axi_rready_i),
      .data_o ({axi_rlast_o, axi_rdata_o})
  );


endmodule  // axi_rd_path
