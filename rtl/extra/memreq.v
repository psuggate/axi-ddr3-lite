`timescale 1ns / 100ps
/**
 * Parses USB packets to generate DDR3 controller requests, and then sends data,
 * or status responses.
 *
 * Todo:
 *  - read/write the status registers of other endpoints ??
 */
module memreq #(
    parameter FIFO_DEPTH = 512,
    parameter DATA_WIDTH = 32,
    localparam MSB = DATA_WIDTH - 1,
    parameter STROBES = DATA_WIDTH / 8,
    localparam SSB = STROBES - 1,
    localparam ADDRESS_WIDTH = 28,
    localparam AZERO = {ADDRESS_WIDTH{1'b0}},
    localparam ASB = ADDRESS_WIDTH - 1,
    localparam ID_WIDTH = 4,
    localparam ISB = ID_WIDTH - 1,
    parameter WR_FRAME_FIFO = 1,  // Avoid "starvation," if slow upstream source
    localparam RD_FRAME_FIFO = 0  // Not useful ??
) (
    input mem_clock,  // DDR3 controller domain
    input mem_reset,

    input bus_clock,  // SPI or USB domain
    input bus_reset,

    // From USB or SPI
    input s_tvalid,
    output s_tready,
    input s_tkeep,
    input s_tlast,
    input [7:0] s_tdata,

    // To USB or SPI
    output m_tvalid,
    input m_tready,
    output m_tkeep,
    output m_tlast,
    output [7:0] m_tdata,

    // Write -address, -data, & -response ports, to/from DDR3 controller
    output awvalid_o,
    input awready_i,
    output [ASB:0] awaddr_o,
    output [ISB:0] awid_o,
    output [7:0] awlen_o,
    output [1:0] awburst_o,

    output wvalid_o,
    input wready_i,
    output wlast_o,
    output [SSB:0] wstrb_o,
    output [MSB:0] wdata_o,

    input bvalid_i,
    output bready_o,
    input [1:0] bresp_i,
    input [ISB:0] bid_i,

    // Read -address & -data ports, to/from the DDR3 controller
    output arvalid_o,
    input arready_i,
    output [ASB:0] araddr_o,
    output [ISB:0] arid_o,
    output [7:0] arlen_o,
    output [1:0] arburst_o,

    input rvalid_i,
    output rready_o,
    input rlast_i,
    input [1:0] rresp_i,
    input [ISB:0] rid_i,
    input [MSB:0] rdata_i
);

  // -- Constants -- //

  `include "axi_defs.vh"

  localparam [7:0] CMD_NOP = 8'h00;
  localparam [7:0] CMD_STORE = 8'h01;
  localparam [7:0] CMD_WDONE = 8'h02;
  localparam [7:0] CMD_WFAIL = 8'h03;
  localparam [7:0] CMD_FETCH = 8'h80;
  localparam [7:0] CMD_RDATA = 8'h81;
  localparam [7:0] CMD_RFAIL = 8'h82;

  localparam CMD_FIFO_WIDTH = ID_WIDTH + ADDRESS_WIDTH + 8 + 1;
  localparam CSB = CMD_FIFO_WIDTH - 1;

  localparam DBITS = $clog2(FIFO_DEPTH);
  localparam DSB = DBITS - 1;

  localparam ST_IDLE = 1;
  localparam ST_WADR = 2;
  localparam ST_WDAT = 4;
  localparam ST_RESP = 8;
  localparam ST_RADR = 16;
  localparam ST_RDAT = 32;
  localparam ST_SEND = 64;
  localparam ST_DONE = 128;

  localparam [3:0] WR_IDLE = 1;
  localparam [3:0] WR_ADDR = 2;
  localparam [3:0] WR_DATA = 4;
  localparam [3:0] WR_RESP = 8;

  localparam [3:0] RD_IDLE = 1;
  localparam [3:0] RD_ADDR = 2;
  localparam [3:0] RD_DATA = 4;
  localparam [3:0] RD_SEND = 8;

  // -- Datapath Signals -- //

  reg [7:0] state, snext;
  reg [3:0] wr, rd;
  reg [4:0] ptr_q, cmd_m, rd_m;
  reg wen_q, stb_q, new_q, cyc_q;
  reg mux_q, sel_q, vld_q, lst_q, idx_q;
  reg [  7:0] res_q;
  reg [ISB:0] rid_q;
  reg [7:0] cmd_q, len_q, len_c, len_m;
  reg [ISB:0] tid_m;
  reg [ASB:0] adr_m;
  reg [ISB:0] tid_q, tid_c;  // 4b
  reg [ASB:0] adr_q, adr_c;  // 28b
  wire mux_enable_w, mux_select_w;
  wire svalid_w, sready_w, fready_w, fvalid_w, rd_mid_w;
  wire [DBITS:0] rd_level_w;
  wire tkeep_w, rvalid_w, cmd_end_w;
  wire bokay_w, wfull_w, rokay_w;
  wire cmd_w, ack_w, rd_w;
  wire wr_cmd_w, wr_ack_w, wr_end_w, rd_cmd_w, rd_ack_w, rd_end_w;
  wire [ISB:0] tid_w, rid_w;
  wire [  7:0] len_w;
  wire [ASB:0] adr_w;
  wire [CSB:0] cdata_w;

  wire x_tvalid, x_tready, x_tlast, y_tvalid, y_tready, y_tkeep, y_tlast;
  wire a_tvalid, a_tready, a_tlast, b_tvalid, b_tready, b_tlast;
  wire z_tvalid, z_tready, z_tkeep, z_tlast, cvalid_w, cready_w;
  wire [SSB:0] a_tkeep, x_tkeep, b_tkeep;
  wire [1:0] b_tuser, y_tuser;
  wire [ISB:0] x_tid, y_tid, a_tid, b_tid;
  wire [7:0] y_tdata, z_tdata;
  wire [MSB:0] x_tdata, b_tdata, a_tdata;

  assign s_tready = sready_w && cready_w && cyc_q;

  // Todo ...
  assign awvalid_o = cmd_m && !rd_m;
  assign awburst_o = BURST_TYPE_INCR;
  assign awlen_o   = len_m;
  assign awid_o    = tid_m;
  assign awaddr_o  = adr_m;

  // Write-buffer (FIFO) assignments, to the DDR3 controller
  assign wvalid_o = wr == WR_DATA && x_tvalid;
  assign wlast_o  = wr == WR_DATA && x_tlast;
  assign wstrb_o  = {STROBES{x_tvalid}};
  assign wdata_o  = x_tdata;

  // Read-address assignments, to the DDR3 controller
  assign arvalid_o = cmd_m && rd_m;
  assign arburst_o = BURST_TYPE_INCR;
  assign arlen_o   = len_m;
  assign arid_o    = tid_m;
  assign araddr_o  = adr_m;

  assign rready_o = rd == RD_DATA && fready_w;

  // -- Parser for Memory Transaction Requests -- //

  // DeMUX for memory requests
  always @* begin
    len_c = len_q;
    tid_c = tid_q;
    adr_c = adr_q;

    case (ptr_q)
      5'h01: len_c = s_tdata;
      5'h02: adr_c[7:0] = s_tdata;
      5'h04: adr_c[15:8] = s_tdata;
      5'h08: adr_c[23:16] = s_tdata;
      5'h10: {tid_c, adr_c[ASB:24]} = s_tdata;
    endcase
  end

  always @(posedge bus_clock) begin
    if (bus_reset) begin
      new_q <= 1'b0;
      cyc_q <= 1'b0;
    end else if (!cyc_q && state == ST_IDLE && s_tvalid && s_tkeep) begin
      new_q <= 1'b1;
      cyc_q <= 1'b1;
    end else begin
      new_q <= 1'b0;
      cyc_q <= s_tvalid && s_tready && s_tlast ? 1'b0 : cyc_q;
    end
  end

  always @(posedge bus_clock) begin
    if (state == ST_IDLE) begin
      ptr_q <= 5'b00001;
      cmd_q <= s_tdata;
      len_q <= 8'bx;
      tid_q <= {ID_WIDTH{1'bx}};
      adr_q <= {ADDRESS_WIDTH{1'bx}};
    end else if (s_tvalid && s_tkeep && s_tready) begin
      ptr_q <= {ptr_q[4:0], 1'b0};
      cmd_q <= cmd_q;
      len_q <= len_c;
      tid_q <= tid_c;
      adr_q <= adr_c;
    end
  end

  always @(posedge bus_clock) begin
    if (bus_reset || stb_q && cready_w) begin
      stb_q <= 1'b0;
    end else if (ptr_q[4]) begin
      stb_q <= 1'b1;
    end
  end

  always @(posedge bus_clock) begin
    if (bus_reset) begin
      wen_q <= 1'b0;
    end else begin
      if (cmd_q[7] == 1'b0 && ptr_q[4] && state != ST_IDLE) begin
        wen_q <= 1'b1;
      end else if (s_tvalid && s_tready && s_tlast) begin
        wen_q <= 1'b0;
      end
    end
  end

  // -- FSM for Memory Requests -- //

  assign cmd_end_w = ptr_q[4];
  assign svalid_w  = s_tvalid & tkeep_w;

  always @(posedge bus_clock) begin
    if (bus_reset) begin
      state <= ST_IDLE;
    end else begin
      case (state)
        ST_IDLE: begin
          if (s_tvalid && s_tready && s_tkeep) begin
            state <= s_tdata[7] ? ST_RADR : ST_WADR;
          end
        end

        // Write States //
        ST_WADR: state <= cmd_end_w ? ST_WDAT : state;
        ST_WDAT: state <= s_tvalid && s_tready && s_tlast ? ST_RESP : state;
        ST_RESP: state <= rvalid_w ? ST_DONE : state;
        // ST_RESP: state <= rvalid_w ? ST_IDLE : state;

        // Read States //
        ST_RADR: state <= cmd_end_w ? ST_RDAT : state;
        ST_RDAT: state <= b_tvalid && b_tready && b_tlast ? ST_SEND : state;
        ST_SEND: state <= m_tvalid && m_tready && m_tlast ? ST_IDLE : state;

        ST_DONE: state <= mux_q || sel_q ? state : ST_IDLE;
      endcase
    end
  end

  // -- Memory-Domain Command & Address Synchronisation -- //

  assign cvalid_w = stb_q;
  assign cdata_w = {cmd_q[7], tid_q, len_q, adr_q};

  // Note: According to the AXI spec., not supposed to have combinational logic
  //   between 'valid' and 'ready' ports, which is why these signals are laid-
  //   out this way.
  assign ack_w = !cmd_m && wr == WR_IDLE && rd == RD_IDLE && (rd_w ? fready_w : x_tvalid);

  assign wr_cmd_w = rd_w == 1'b0 && cmd_w && ack_w;
  assign wr_ack_w = awvalid_o && awready_i;
  assign wr_end_w = x_tvalid && x_tready && x_tlast;

  assign rd_cmd_w = rd_w == 1'b1 && cmd_w && ack_w;
  assign rd_ack_w = arvalid_o && arready_i;
  assign rd_end_w = fvalid_w && fready_w && rlast_i;

  always @(posedge mem_clock) begin
    if (mem_reset || wr_ack_w || rd_ack_w) begin
      cmd_m <= 1'b0;
      {rd_m, tid_m, len_m, adr_m} <= {CMD_FIFO_WIDTH{1'bx}};
    end else if (cmd_w && ack_w) begin
      cmd_m <= 1'b1;
      rd_m  <= rd_w;
      tid_m <= tid_w;
      len_m <= len_w;
      adr_m <= adr_w;
    end
  end

  axis_afifo #(
      .WIDTH(CMD_FIFO_WIDTH),
      .TLAST(0),
      .ABITS(4)
  ) U_CFIFO1 (
      .aresetn(~bus_reset),

      .s_aclk  (bus_clock),
      .s_tvalid(cvalid_w),
      .s_tready(cready_w),
      .s_tlast (1'b1),
      .s_tdata (cdata_w),

      .m_aclk  (mem_clock),
      .m_tvalid(cmd_w),
      .m_tready(ack_w),
      .m_tlast (),
      .m_tdata ({rd_w, tid_w, len_w, adr_w})
  );

  // -- Write-Port, Memory-Domain FSM -- //

  assign x_tready = wr == WR_DATA ? wready_i : 1'b0;

  assign tkeep_w  = wen_q;  // state == ST_WDAT;
  assign bokay_w  = bresp_i == RESP_OKAY;

  always @(posedge mem_clock) begin
    if (mem_reset) begin
      wr <= WR_IDLE;
    end else begin
      case (wr)
        WR_IDLE: wr <= wr_cmd_w ? WR_ADDR : wr;
        WR_ADDR: wr <= wr_ack_w ? WR_DATA : wr;
        WR_DATA: wr <= wr_end_w ? WR_RESP : wr;
        WR_RESP: begin
          if (bvalid_i && bready_o) begin
            wr <= WR_IDLE;
          end
        end
        default: wr <= 'bx;
      endcase
    end
  end

  // -- Read-Port, Memory-Domain FSM -- //

  assign fvalid_w = rd == RD_DATA && rvalid_i;
  assign rd_mid_w = rd_level_w[DSB];

  always @(posedge mem_clock) begin
    if (mem_reset) begin
      rd <= RD_IDLE;
    end else begin
      case (rd)
        RD_IDLE: rd <= rd_cmd_w ? RD_ADDR : rd;
        RD_ADDR: rd <= rd_ack_w ? RD_DATA : rd;
        RD_DATA: rd <= rd_end_w ? RD_SEND : rd;
        RD_SEND: rd <= rd_mid_w ? rd : RD_IDLE;
        default: rd <= 'bx;
      endcase
    end
  end

  // -- Multiplexor to the SPI or USB Encoder -- //

  assign z_tvalid = vld_q;
  assign z_tkeep  = vld_q;
  assign z_tlast  = lst_q;
  assign z_tdata  = idx_q ? rid_q : res_q;

  always @(posedge bus_clock) begin
    if (bus_reset) begin
      {mux_q, sel_q, idx_q, vld_q, lst_q} <= 5'h00;
      res_q <= 8'bx;
      rid_q <= 8'bx;
    end else begin
      case (state)
        ST_IDLE: begin
          {mux_q, sel_q, idx_q, vld_q, lst_q} <= 5'h00;
        end

        ST_RESP: begin
          if (rvalid_w) begin
            {mux_q, sel_q, idx_q, vld_q, lst_q} <= 5'h1a;
            res_q <= bokay_w ? CMD_WDONE : CMD_WFAIL;
            rid_q <= {{(8 - ID_WIDTH) {1'b0}}, rid_w};
          end
        end

        ST_DONE: begin
          if (z_tready && vld_q && !idx_q) begin
            // Send the 2nd byte of the write-response
            {mux_q, sel_q, idx_q, vld_q, lst_q} <= 5'h1f;
          end else if (z_tready) begin
            {mux_q, sel_q, idx_q, vld_q, lst_q} <= 5'h00;
          end
        end

        ST_RDAT: begin
          if (!idx_q && !vld_q && y_tvalid && y_tready) begin
            {mux_q, sel_q, idx_q, vld_q, lst_q} <= 5'h1a;
            res_q <= b_tuser == RESP_OKAY ? CMD_RDATA : CMD_RFAIL;
            rid_q <= {{(8 - ID_WIDTH) {1'b0}}, y_tid};
          end else if (!idx_q && vld_q) begin
            {mux_q, sel_q, idx_q, vld_q, lst_q} <= 5'h1e;
          end else if (idx_q && vld_q) begin
            {mux_q, sel_q, idx_q, vld_q, lst_q} <= 5'h14;
          end else begin
            // Todo: this is the same as the previous case !?
            {mux_q, sel_q, idx_q, vld_q, lst_q} <= 5'h14;
          end
        end

        ST_SEND: begin
          {sel_q, idx_q, vld_q, lst_q} <= 4'h4;
          if (m_tvalid && m_tready && m_tlast) begin
            mux_q <= 1'b0;
          end else begin
            mux_q <= 1'b1;
          end
        end

        default: begin
          {mux_q, sel_q, idx_q, vld_q, lst_q} <= {mux_q, sel_q, idx_q, vld_q, lst_q};
        end

      endcase
    end
  end

  // -- Write Datapath -- //

  axis_adapter #(
      .S_DATA_WIDTH(8),
      .S_KEEP_ENABLE(1),
      .S_KEEP_WIDTH(1),
      .M_DATA_WIDTH(DATA_WIDTH),
      .M_KEEP_ENABLE(1),
      .M_KEEP_WIDTH(STROBES),
      .ID_ENABLE(1),
      .ID_WIDTH(ID_WIDTH),
      .DEST_ENABLE(0),
      .DEST_WIDTH(1),
      .USER_ENABLE(0),
      .USER_WIDTH(1)
  ) U_ADAPT1 (
      .clk(bus_clock),
      .rst(bus_reset),

      .s_axis_tvalid(svalid_w),
      .s_axis_tready(sready_w),
      .s_axis_tkeep(tkeep_w),
      .s_axis_tlast(s_tlast),
      .s_axis_tid(tid_q),
      .s_axis_tdest(1'b0),
      .s_axis_tuser(1'b0),
      .s_axis_tdata(s_tdata),  // AXI input

      .m_axis_tvalid(a_tvalid),
      .m_axis_tready(a_tready),
      .m_axis_tkeep(a_tkeep),
      .m_axis_tlast(a_tlast),
      .m_axis_tid(a_tid),
      .m_axis_tdest(),
      .m_axis_tuser(),
      .m_axis_tdata(a_tdata)  // AXI output
  );

  axis_async_fifo #(
      .DEPTH(FIFO_DEPTH),
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_ENABLE(1),
      .KEEP_WIDTH(STROBES),
      .LAST_ENABLE(1),
      .ID_ENABLE(1),
      .ID_WIDTH(ID_WIDTH),
      .DEST_ENABLE(0),
      .DEST_WIDTH(1),
      .USER_ENABLE(0),
      .USER_WIDTH(1),
      .RAM_PIPELINE(1),
      .OUTPUT_FIFO_ENABLE(0),
      .FRAME_FIFO(WR_FRAME_FIFO),
      .USER_BAD_FRAME_VALUE(0),
      .USER_BAD_FRAME_MASK(0),
      .DROP_BAD_FRAME(0),
      .DROP_WHEN_FULL(0)
  ) U_WRFIFO1 (
      .s_clk(bus_clock),
      .s_rst(bus_reset),

      .s_axis_tvalid(a_tvalid),
      .s_axis_tready(a_tready),
      .s_axis_tkeep(a_tkeep),
      .s_axis_tlast(a_tlast),
      .s_axis_tdata(a_tdata),  // AXI input
      .s_axis_tid(a_tid),
      .s_axis_tdest(1'b0),
      .s_axis_tuser(1'b0),

      .m_clk(mem_clock),
      .m_rst(mem_reset),

      .m_axis_tvalid(x_tvalid),
      .m_axis_tready(x_tready),
      .m_axis_tkeep(x_tkeep),
      .m_axis_tlast(x_tlast),
      .m_axis_tdata(x_tdata),  // AXI output
      .m_axis_tid(x_tid),
      .m_axis_tdest(),
      .m_axis_tuser(),

      .s_pause_req(1'b0),
      .s_pause_ack(),
      .m_pause_req(1'b0),
      .m_pause_ack(),

      .s_status_depth(),  // Status
      .s_status_depth_commit(),
      .s_status_overflow(),
      .s_status_bad_frame(),
      .s_status_good_frame(),
      .m_status_depth(),  // Status
      .m_status_depth_commit(),
      .m_status_overflow(),
      .m_status_bad_frame(),
      .m_status_good_frame()
  );

  // `define __use_potatio
`ifdef __use_potatio

  // Write-responses FIFO
  axis_afifo #(
      .WIDTH(ID_WIDTH + 1),
      .TLAST(0),
      .ABITS(4)
  ) U_BFIFO1 (
      .aresetn (~bus_reset),
      .s_aclk  (mem_clock),
      .s_tvalid(bvalid_i),
      .s_tready(bready_o),
      .s_tlast (1'b1),
      .s_tdata ({bokay_w, bid_i}),
      .m_aclk  (bus_clock),
      .m_tvalid(rvalid_w),
      .m_tready(state == ST_RESP),
      .m_tlast (),
      .m_tdata ({rokay_w, rid_w})
  );

`else  /* !__use_potatio */

  axis_async_fifo #(
      .DEPTH(16),
      .DATA_WIDTH(1),
      .KEEP_ENABLE(0),
      .KEEP_WIDTH(1),
      .LAST_ENABLE(0),
      .ID_ENABLE(1),
      .ID_WIDTH(ID_WIDTH),
      .DEST_ENABLE(0),
      .DEST_WIDTH(1),
      .USER_ENABLE(0),
      .USER_WIDTH(1),
      .RAM_PIPELINE(0),
      .OUTPUT_FIFO_ENABLE(0),
      .FRAME_FIFO(0),
      .USER_BAD_FRAME_VALUE(0),
      .USER_BAD_FRAME_MASK(0),
      .DROP_BAD_FRAME(0),
      .DROP_WHEN_FULL(0)
  ) U_BFIFO1 (
      .s_clk(mem_clock),
      .s_rst(mem_reset),

      .s_axis_tvalid(bvalid_i),  // AXI input: 32b, MEM domain
      .s_axis_tready(bready_o),
      .s_axis_tkeep(1'b1),
      .s_axis_tlast(1'b1),
      .s_axis_tid(bid_i),
      .s_axis_tdest(1'b0),
      .s_axis_tuser(1'b0),
      .s_axis_tdata(bokay_w),

      .m_clk(bus_clock),
      .m_rst(bus_reset),

      .m_axis_tvalid(rvalid_w),  // AXI output: 8b, BUS domain
      .m_axis_tready(state == ST_RESP),
      .m_axis_tkeep(),
      .m_axis_tlast(),
      .m_axis_tid(rid_w),
      .m_axis_tdest(),
      .m_axis_tuser(),
      .m_axis_tdata(rokay_w),

      .s_pause_req(1'b0),
      .s_pause_ack(),
      .m_pause_req(1'b0),
      .m_pause_ack(),

      .s_status_depth(),  // Status
      .s_status_depth_commit(),
      .s_status_overflow(),
      .s_status_bad_frame(),
      .s_status_good_frame(),
      .m_status_depth(),  // Status
      .m_status_depth_commit(),
      .m_status_overflow(),
      .m_status_bad_frame(),
      .m_status_good_frame()
  );

`endif  /* !__use_potatio */

  // -- Read Datapath -- //

  axis_async_fifo #(
      .DEPTH(FIFO_DEPTH),
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_ENABLE(1),
      .KEEP_WIDTH(STROBES),
      .LAST_ENABLE(1),
      .ID_ENABLE(1),
      .ID_WIDTH(ID_WIDTH),
      .DEST_ENABLE(0),
      .DEST_WIDTH(1),
      .USER_ENABLE(1),
      .USER_WIDTH(2),
      .RAM_PIPELINE(1),
      .OUTPUT_FIFO_ENABLE(0),
      .FRAME_FIFO(RD_FRAME_FIFO),
      .USER_BAD_FRAME_VALUE(0),
      .USER_BAD_FRAME_MASK(0),
      .DROP_BAD_FRAME(0),
      .DROP_WHEN_FULL(0)
  ) U_RDFIFO1 (
      .s_clk(mem_clock),
      .s_rst(mem_reset),

      .s_axis_tvalid(fvalid_w),  // AXI input: 32b, MEM domain
      .s_axis_tready(fready_w),
      .s_axis_tkeep({STROBES{rvalid_i}}),
      .s_axis_tlast(rlast_i),
      .s_axis_tid(rid_i),
      .s_axis_tdest(1'b0),
      .s_axis_tuser(rresp_i),
      .s_axis_tdata(rdata_i),

      .m_clk(bus_clock),
      .m_rst(bus_reset),

      .m_axis_tvalid(b_tvalid),  // AXI output: 8b, BUS domain
      .m_axis_tready(b_tready),
      .m_axis_tkeep(b_tkeep),
      .m_axis_tlast(b_tlast),
      .m_axis_tid(b_tid),
      .m_axis_tdest(),
      .m_axis_tuser(b_tuser),
      .m_axis_tdata(b_tdata),

      .s_pause_req(1'b0),
      .s_pause_ack(),
      .m_pause_req(1'b0),
      .m_pause_ack(),

      .s_status_depth(rd_level_w),  // Status
      .s_status_depth_commit(),
      .s_status_overflow(),
      .s_status_bad_frame(),
      .s_status_good_frame(),
      .m_status_depth(),  // Status
      .m_status_depth_commit(),
      .m_status_overflow(),
      .m_status_bad_frame(),
      .m_status_good_frame()
  );

  axis_adapter #(
      .S_DATA_WIDTH(DATA_WIDTH),
      .S_KEEP_ENABLE(1),
      .S_KEEP_WIDTH(STROBES),
      .M_DATA_WIDTH(8),
      .M_KEEP_ENABLE(1),
      .M_KEEP_WIDTH(1),
      .ID_ENABLE(1),
      .ID_WIDTH(ID_WIDTH),
      .DEST_ENABLE(0),
      .DEST_WIDTH(1),
      .USER_ENABLE(1),
      .USER_WIDTH(2)
  ) U_ADAPT2 (
      .clk(bus_clock),
      .rst(bus_reset),

      .s_axis_tvalid(b_tvalid),  // AXI input: 32b
      .s_axis_tready(b_tready),
      .s_axis_tkeep({STROBES{b_tvalid}}),
      .s_axis_tlast(b_tlast),
      .s_axis_tdata(b_tdata),
      .s_axis_tid(b_tid),
      .s_axis_tdest(1'b0),
      .s_axis_tuser(b_tuser),

      .m_axis_tvalid(y_tvalid),  // AXI output: 8b
      .m_axis_tready(y_tready),
      .m_axis_tkeep(y_tkeep),
      .m_axis_tlast(y_tlast),
      .m_axis_tid(y_tid),
      .m_axis_tdest(),
      .m_axis_tuser(y_tuser),
      .m_axis_tdata(y_tdata)
  );

  // Multiplexor to the SPI or USB Encoder
  axis_mux #(
      .S_COUNT(2),
      .DATA_WIDTH(8),
      .KEEP_ENABLE(1),
      .KEEP_WIDTH(1),
      .ID_ENABLE(0),
      .ID_WIDTH(1),
      .DEST_ENABLE(0),
      .DEST_WIDTH(1),
      .USER_ENABLE(0),
      .USER_WIDTH(1)
  ) U_MUX1 (
      .clk(bus_clock),
      .rst(bus_reset),

      .enable(mux_q),
      .select(sel_q),

      .s_axis_tvalid({z_tvalid, y_tvalid}), // AXI input: 2x 8b
      .s_axis_tready({z_tready, y_tready}),
      .s_axis_tkeep ({z_tkeep, y_tkeep}),
      .s_axis_tlast ({z_tlast, y_tlast}),
      .s_axis_tuser (2'bx),
      .s_axis_tid   (2'bx),
      .s_axis_tdest (2'bx),
      .s_axis_tdata ({z_tdata, y_tdata}),

      .m_axis_tvalid(m_tvalid), // AXI output: 8b
      .m_axis_tready(m_tready),
      .m_axis_tkeep (m_tkeep),
      .m_axis_tlast (m_tlast),
      .m_axis_tuser (),
      .m_axis_tid   (),
      .m_axis_tdest (),
      .m_axis_tdata (m_tdata)
  );

`ifdef __icarus
  //
  //  Simulation Only
  ///

  reg [39:0] dbg_state, dbg_wr, dbg_rd;
  reg [47:0] dbg_cmd, dbg_res;

  wire [7:0] dbg_mdata = m_tvalid & m_tready & m_tkeep ? m_tdata : 8'bz;
  wire [7:0] dbg_sdata = s_tvalid & s_tready & s_tkeep ? s_tdata : 8'bz;

  always @* begin
    case (state)
      ST_IDLE: dbg_state = "IDLE";
      ST_WADR: dbg_state = "WADR";
      ST_WDAT: dbg_state = "WDAT";
      ST_RESP: dbg_state = "RESP";
      ST_RADR: dbg_state = "RADR";
      ST_RDAT: dbg_state = "RDAT";
      ST_SEND: dbg_state = "SEND";
      ST_DONE: dbg_state = "DONE";
      default: dbg_state = " ?? ";
    endcase
  end

  always @* begin
    case (wr)
      WR_IDLE: dbg_wr = "IDLE";
      WR_ADDR: dbg_wr = "ADDR";
      WR_DATA: dbg_wr = "DATA";
      WR_RESP: dbg_wr = "RESP";
      default: dbg_wr = " ?? ";
    endcase
    case (rd)
      RD_IDLE: dbg_rd = "IDLE";
      RD_ADDR: dbg_rd = "ADDR";
      RD_DATA: dbg_rd = "DATA";
      RD_SEND: dbg_rd = "SEND";
      default: dbg_rd = " ?? ";
    endcase
  end

  always @* begin
    case (cmd_q)
      CMD_NOP:   dbg_cmd = " NOP ";
      CMD_STORE: dbg_cmd = "STORE";
      CMD_WDONE: dbg_cmd = "WDONE";
      CMD_WFAIL: dbg_cmd = "WFAIL";
      CMD_FETCH: dbg_cmd = "FETCH";
      CMD_RDATA: dbg_cmd = "RDATA";
      CMD_RFAIL: dbg_cmd = "RFAIL";
      default:   dbg_cmd = " ??? ";
    endcase
    case (res_q)
      CMD_NOP:   dbg_res = " NOP ";
      CMD_STORE: dbg_res = "STORE";
      CMD_WDONE: dbg_res = "WDONE";
      CMD_WFAIL: dbg_res = "WFAIL";
      CMD_FETCH: dbg_res = "FETCH";
      CMD_RDATA: dbg_res = "RDATA";
      CMD_RFAIL: dbg_res = "RFAIL";
      default:   dbg_res = " ??? ";
    endcase
  end

`endif  /* __icarus */

endmodule  /* memreq */
