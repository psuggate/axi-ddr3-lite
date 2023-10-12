`timescale 1ns / 100ps
/**
 * Converts AXI4 requests into simple memory-controller commands.
 *
 * Notes:
 *  - assumes that the AXI4 interface converts write-data into 128-bit chunks,
 *    padding as required;
 *  - read data will also be a (continuous) stream of 128-bit chunks, so the
 *    AXI4 interface will have to drop any (unwanted) trailing data, if not
 *    required;
 *  - assumes that the memory controller and the AXI4 bus are within the same
 *    clock-domain;
 *
 * Copyright 2023, Patrick Suggate.
 *
 */
module ddr3_axi_ctrl (
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

    mem_store_o,
    mem_fetch_o,
    mem_accept_i,
    mem_error_i,
    mem_req_id_o,
    mem_addr_o,

    mem_valid_o,
    mem_ready_i,
    mem_last_o,
    mem_wrmask_o,
    mem_wrdata_o,

    mem_valid_i,
    mem_ready_o,
    mem_last_i,
    mem_resp_id_i,
    mem_rddata_i
);

  parameter DDR_FREQ_MHZ = 100;
  parameter DDR_WR_LATENCY = 6;
  parameter DDR_RD_LATENCY = 5;
  localparam DDR_BURST_LEN = 4;

  localparam DDR_BANK_BITS = 3;
  localparam BSB = DDR_BANK_BITS - 1;
  parameter DDR_COL_BITS = 9;
  localparam CSB = DDR_COL_BITS - 1;
  parameter DDR_ROW_BITS = 15;
  localparam RSB = DDR_ROW_BITS - 1;

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

// If the memory controller is idle (and both datapaths), send any request
// straight to the memory-controller (if 'FAST_PATH_ENABLE == 1')
parameter FAST_PATH_ENABLE = 1;


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
  input axi_arvalid_i;  // AXI4 Read Address Port
  output axi_arready_o;
  input [ASB:0] axi_araddr_i;
  input [ISB:0] axi_arid_i;
  input [7:0] axi_arlen_i;
  input [1:0] axi_arburst_i;
  input axi_rready_i;  // AXI4 Read Data Port
  output axi_rvalid_o;
  output [MSB:0] axi_rdata_o;
  output [1:0] axi_rresp_o;
  output [ISB:0] axi_rid_o;
  output axi_rlast_o;

  output mem_store_o;
  output mem_fetch_o;
  input mem_accept_i;
  input mem_error_i;
  output [ISB:0] mem_req_id_o;
  output [MSB:0] mem_addr_o;

  output mem_valid_o;
  input mem_ready_i;
  output mem_last_o;
  output [SSB:0] mem_wrmask_o;
  output [MSB:0] mem_wrdata_o;

  input mem_valid_i;
  output mem_ready_o;
  input mem_last_i;
  input [ISB:0] mem_resp_id_i;
  input [MSB:0] mem_rddata_i;


  // -- Constants -- //

  localparam [1:0] BURST_INCR = 2'b01;

  // todo: this is but a sketch ...
  localparam COMMAND_WIDTH = 4 + 1 + WIDTH;

localparam TRANS = 4;
localparam TSB = TRANS - 1;


  // -- Turn AXI4 Commands into Memory-Controller Commands -- //

  reg awready, wready, wpending, arready;
  wire wdf_ready, wrf_ready, cmd_ready, mem_store_w, mem_accept, mem_ready;


  // assign axi_awready_o = awready;
  // assign axi_arready_o = arready;

localparam ST_IDLE = 4'b0000;
localparam ST_BUSY = 4'b1000;

reg [3:0] state = ST_IDLE;

// Current, Next, Read, Write (Transaction ID's)
reg [TSB:0] trid;
wire [TSB:0] ctrid, ntrid, rtrid, wtrid;

assign ntrid = trid + 1;

always @(posedge clock) begin
  if (reset) begin
    trid <= {TRANS{1'b0}};
    state <= ST_IDLE;
  end else begin
    case (state)
        ST_IDLE: begin
        end

        ST_BUSY: begin
          // Can not queue any more AXI requests
        end

        default: begin
          $error("AX:%10t: CTRL state machine failure!", $time);
          $fatal;
        end
    endcase
  end
end


  // -- Write Requests -- //

  // todo: there are a bunch of different FSM's that are needed ??
  // todo: add 'skid-buffers' to each channel ??
  // todo: for unaligned writes, padding & masks will need to be inserted -- here
  //   or in the memory controller ??
  // todo: how to handle close-together read & write requests, since read/write
  //   ordering may not be easy to determine (by AXI4 masters) ??
  always @(posedge clock) begin
    if (reset) begin
      awready  <= 1'b0;
      wpending <= 1'b0;
      wready   <= 1'b0;
    end else begin
      // Can accept commands as long as there is space in both the command FIFO
      // and the write-data FIFO.
      // todo: handle simultaneous read & write requests
      awready <= cmd_ready & wdf_ready;

      // todo: unnecessarily restrictive -- does not allow for multiple writes
      //   "in-flight" ??
      if (awready && axi_awvalid_i) begin
        wpending <= 1'b1;
      end else if (wready && axi_wvalid_i && axi_wlast_i) begin
        wpending <= 1'b0;
      end

      wready <= wpending & wdf_ready;
    end
  end


  // -- Read Requests -- //

  always @(posedge clock) begin
    if (reset) begin
      arready <= 1'b0;
    end else begin
      // Can accept commands as long as there is space in both the command FIFO
      // and the read-data FIFO.
      // todo: handle simultaneous read & write requests
      arready <= cmd_ready & mem_ready;
    end
  end


  // -- Queue the Commands to the Memory-Controller -- //

  reg [3:0] req_id;
  reg req_we, valid, store;
  reg [ASB:0] req_ad;

  wire cmd_valid, cmd_queued, mem_accept_w;
  reg [COMMAND_WIDTH-1:0] command_q;
  wire [COMMAND_WIDTH-1:0] command_w, command_a;


  assign mem_req_id_o = command_a[COMMAND_WIDTH-1:COMMAND_WIDTH-3];

  always @(posedge clock) begin
    command_q <= command_w;
  end


  // Do not enqueue commands when the command-FIFO is full.
  reg cmd_block;

  assign command_w = {req_id, valid, store, req_ad};
  assign cmd_valid = axi_awvalid_i || axi_arvalid_i;

  always @(posedge clock) begin
    if (reset) begin
      cmd_block <= 1'b1;
      // cmd_valid <= 1'b0;
      // cmd_data  <= 'bx;
    end else begin
      cmd_block <= ~cmd_ready;
      // if (axi_awvalid_i && cmd_ready) begin
    end
  end


  // -- AXI Interface to Memory Controller for Write-Data -- //

  wire wr_store, wr_accept, wr_valid, wr_ready;
  wire [ISB:0] wr_reqid;
  wire [ASB:0] wr_addr;

  wire rd_fetch, rd_accept, rd_ready;
  wire [ISB:0] rd_reqid;
  wire [ASB:0] rd_addr;

  assign mem_store_o = wr_store;
  assign mem_fetch_o = rd_fetch;
  assign mem_req_id_o = wr_store ? wr_reqid : rd_reqid;
  assign mem_addr_o = wr_store ? wr_addr : rd_addr;

  assign wr_accept = mem_accept_i & wr_store;
  assign rd_accept = mem_accept_i & rd_fetch;

  axi_wr_path #(
      .ADDRS(ADDRS),
      .WIDTH(WIDTH),
      .MASKS(MASKS),
      .AXI_ID_WIDTH(AXI_ID_WIDTH),
      .CTRL_FIFO_DEPTH(CTRL_FIFO_DEPTH),
      .DATA_FIFO_DEPTH(DATA_FIFO_DEPTH)
  ) axi_wr_path_inst (
      .clock(clock),
      .reset(reset),

      .axi_awvalid_i(axi_awvalid_i),  // AXI4 Write Address Port
      .axi_awready_o(axi_awready_o),
      .axi_awid_i(axi_awid_i),
      .axi_awlen_i(axi_awlen_i),
      .axi_awburst_i(axi_awburst_i),
      .axi_awaddr_i(axi_awaddr_i),

      .axi_wvalid_i(axi_wvalid_i),  // AXI4 Write Data Port
      .axi_wready_o(axi_wready_o),
      .axi_wlast_i (axi_wlast_i),
      .axi_wstrb_i (axi_wstrb_i),
      .axi_wdata_i (axi_wdata_i),

      .axi_bvalid_o(axi_bvalid_o),  // AXI4 Write Response Port
      .axi_bready_i(axi_bready_i),
      .axi_bid_o(axi_bid_o),
      .axi_bresp_o(axi_bresp_o),

      .mem_store_o (wr_store),
      .mem_accept_i(wr_accept),
      .mem_wrid_o  (wr_reqid),
      .mem_addr_o  (wr_addr),

      .mem_valid_o(mem_valid_o),
      .mem_ready_i(mem_ready_i),
      .mem_last_o (mem_last_o),
      .mem_strb_o (mem_wrmask_o),
      .mem_data_o (mem_wrdata_o)
  );


  // -- AXI Interface to Memory Controller for Read-Data -- //

  axi_rd_path #(
      .ADDRS(ADDRS),
      .WIDTH(WIDTH),
      .MASKS(MASKS),
      .AXI_ID_WIDTH(AXI_ID_WIDTH),
      .CTRL_FIFO_DEPTH(CTRL_FIFO_DEPTH),
      .DATA_FIFO_DEPTH(DATA_FIFO_DEPTH)
  ) axi_rd_path_inst (
      .clock(clock),
      .reset(reset),

      .axi_arvalid_i(axi_arvalid_i),
      .axi_arready_o(axi_arready_o),
      .axi_arid_i(axi_arid_i),
      .axi_arlen_i(axi_arlen_i),
      .axi_arburst_i(axi_arburst_i),
      .axi_araddr_i(axi_araddr_i),

      .axi_rvalid_o(axi_rvalid_o),
      .axi_rready_i(axi_rready_i),
      .axi_rlast_o(axi_rlast_o),
      .axi_rresp_o(axi_rresp_o),
      .axi_rid_o(axi_rid_o),
      .axi_rdata_o(axi_rdata_o),

      .mem_fetch_o (rd_fetch),
      .mem_accept_i(rd_accept),
      .mem_rdid_o  (rd_reqid),
      .mem_addr_o  (rd_addr),

      .mem_valid_i(mem_valid_i),
      .mem_ready_o(mem_ready_o),
      .mem_last_i(mem_last_i),  // todo: ...
      .mem_rdid_i(mem_resp_id_i),
      .mem_data_i(mem_rddata_i)
  );


  // -- AXI4 Transaction Response FIFO -- //

  wire trf_empty_n, trf_full_n, trf_push, trf_pop;
  wire [ISB:0] trid_a, trid_w;

  assign trid_a = rd_fetch ? rd_reqid : (wr_store ? wr_reqid : {AXI_ID_WIDTH{1'bx}}) ;

  assign trf_push = (mem_fetch_o | mem_store_o) & mem_accept_i;
  assign trf_pop = (axi_rvalid_o & axi_rready_i & axi_rlast_o) | (axi_bvalid_o & axi_bready_i) ;


  // todo: store the transaction ID of each request that is sent to the memory
  //   controller, and when each request completes, generate the appropriate
  //   AXI4 response.
  // todo: perhaps this ordering should be left up to the memory controller ??
  sync_fifo #(
      .WIDTH (AXI_ID_WIDTH),
      .ABITS (CBITS),
      .OUTREG(CTRL_FIFO_BLOCK)
  ) response_fifo_inst (
      .clock  (clock),
      .reset  (reset),

      .valid_i(trf_push),
      .ready_o(trf_full_n),
      .data_i (trid_a),

      .valid_o(trf_empty_n),
      .ready_i(trf_pop),
      .data_o (trid_w)
  );


`ifdef __icarus
  always @(posedge clock) begin
    if (reset);
    else begin
      if (axi_awvalid_i && axi_awburst_i != BURST_INCR) begin
        $error("%10t: Only 'INCR' WRITE bursts are supported", $time);
        $fatal;
      end
      if (axi_arvalid_i && axi_arburst_i != BURST_INCR) begin
        $error("%10t: Only 'INCR' READ bursts are supported", $time);
        $fatal;
      end
    end
  end
`endif


endmodule  // ddr3_axi_ctrl
