`timescale 1ns / 100ps
module generic_ddr3_phy (
    clock,
    reset,

    clk_ddr,

    cfg_valid_i,
    cfg_data_i,

    dfi_cke_i,
    dfi_rst_ni,
    dfi_cs_ni,
    dfi_ras_ni,
    dfi_cas_ni,
    dfi_we_ni,
    dfi_odt_i,
    dfi_bank_i,
    dfi_addr_i,
    dfi_wren_i,
    dfi_mask_i,
    dfi_data_i,
    dfi_rden_i,
    dfi_valid_o,
    dfi_data_o,

    ddr3_ck_po,
    ddr3_ck_no,
    ddr3_cke_o,
    ddr3_rst_no,
    ddr3_cs_no,
    ddr3_ras_no,
    ddr3_cas_no,
    ddr3_we_no,
    ddr3_odt_o,
    ddr3_ba_o,
    ddr3_a_o,
    ddr3_dm_o,
    ddr3_dqs_pio,
    ddr3_dqs_nio,
    ddr3_dq_io
);

  parameter DEFAULT_CL = 6;  // According to JEDEC spec, for DLL=off mode
  parameter DEFAULT_CWL = 6;

  parameter DDR3_WIDTH = 16;
  parameter DDR3_MASKS = DDR3_WIDTH / 8;

  localparam MSB = DDR3_WIDTH - 1;
  localparam QSB = DDR3_MASKS - 1;

  localparam DSB = DDR3_WIDTH + MSB;
  localparam SSB = DDR3_MASKS + QSB;

  parameter ADDR_BITS = 14;
  localparam ASB = ADDR_BITS - 1;

  parameter MAX_RW_LATENCY = 12;  // Maximum 'CL'/'CWL'
  localparam CSB = MAX_RW_LATENCY - 1;


  input clock;
  input reset;

  input clk_ddr;  // Same phase, but twice freq of 'clock'

  input cfg_valid_i;
  input [31:0] cfg_data_i;

  input dfi_cke_i;
  input dfi_rst_ni;
  input dfi_cs_ni;
  input dfi_ras_ni;
  input dfi_cas_ni;
  input dfi_we_ni;
  input dfi_odt_i;

  input [2:0] dfi_bank_i;
  input [ASB:0] dfi_addr_i;

  input dfi_wren_i;
  input [SSB:0] dfi_mask_i;
  input [DSB:0] dfi_data_i;

  input dfi_rden_i;
  output dfi_valid_o;
  output [DSB:0] dfi_data_o;

  output ddr3_ck_po;
  output ddr3_ck_no;
  output ddr3_cke_o;
  output ddr3_rst_no;
  output ddr3_cs_no;
  output ddr3_ras_no;
  output ddr3_cas_no;
  output ddr3_we_no;
  output ddr3_odt_o;
  output [2:0] ddr3_ba_o;
  output [ASB:0] ddr3_a_o;
  output [QSB:0] ddr3_dm_o;
  inout [QSB:0] ddr3_dqs_pio;
  inout [QSB:0] ddr3_dqs_nio;
  inout [MSB:0] ddr3_dq_io;


  reg dqs_t, dq_t;
  reg [QSB:0] dqs_p, dqs_n, dm_q;
  reg [MSB:0] dq_q;
  reg cke_q, rst_nq, cs_nq;
  reg ras_nq, cas_nq, we_nq, odt_q;
  reg [2:0] ba_q;

  reg valid_q;
  reg [ASB:0] addr_q;
  reg [DSB:0] data_q;


  // -- DFI Read-Data Signal Assignments -- //

  assign dfi_valid_o  = valid_q;
  assign dfi_data_o   = data_q;


  // -- DDR3 Signal Assignments -- //

  assign ddr3_ck_po   = ~clock;
  assign ddr3_ck_no   = clock;

  assign ddr3_cke_o   = cke_q;
  assign ddr3_rst_no  = rst_nq;
  assign ddr3_cs_no   = cs_nq;
  assign ddr3_ras_no  = ras_nq;
  assign ddr3_cas_no  = cas_nq;
  assign ddr3_we_no   = we_nq;
  assign ddr3_odt_o   = odt_q;
  assign ddr3_ba_o    = ba_q;
  assign ddr3_a_o     = addr_q;

  assign ddr3_dqs_pio = dqs_t ? {DDR3_MASKS{1'bz}} : dqs_p;
  assign ddr3_dqs_nio = dqs_t ? {DDR3_MASKS{1'bz}} : dqs_n;
  assign ddr3_dm_o    = dm_q;
  assign ddr3_dq_io   = dq_t ? {DDR3_WIDTH{1'bz}} : dq_q;


  // -- DFI Configuration -- //

  reg [3:0] rd_lat_q, wr_lat_q;

  always @(posedge clock)
    if (reset) begin
      rd_lat_q <= DEFAULT_CL - 2;
    end else if (cfg_valid_i) begin
      rd_lat_q <= cfg_data_i[11:8];
    end

  always @(posedge clock) begin
    if (reset) begin
      wr_lat_q <= DEFAULT_CWL - 2;
    end else if (cfg_valid_i) begin
      wr_lat_q <= cfg_data_i[15:12];  // todo ...
    end
  end


  // -- DDR3 Command Signals -- //

  // todo: polarities of the 'n' signals?
  always @(posedge clock) begin
    if (reset) begin
      cke_q  <= 1'b0;
      rst_nq <= 1'b0;
      cs_nq  <= 1'b0;  // todo: 1'b1 ??
      ras_nq <= 1'b0;  // todo: 1'b1 ??
      cas_nq <= 1'b0;  // todo: 1'b1 ??
      we_nq  <= 1'b0;  // todo: 1'b1 ??
      ba_q   <= 3'b0;
      addr_q <= {ADDR_BITS{1'b0}};
      odt_q  <= 1'b0;
    end else begin
      cke_q  <= dfi_cke_i;
      rst_nq <= dfi_rst_ni;
      cs_nq  <= dfi_cs_ni;
      ras_nq <= dfi_ras_ni;
      cas_nq <= dfi_cas_ni;
      we_nq  <= dfi_we_ni;
      odt_q  <= dfi_odt_i;
      ba_q   <= dfi_bank_i;
      addr_q <= dfi_addr_i;
    end
  end


  // -- DDR3 Data Strobes -- //

  localparam WRITE_SHIFT_WIDTH = DDR3_WIDTH * 2 + DDR3_MASKS * 2 + 1;
  localparam WRITE_SHIFT_ADDRS = $clog2(MAX_RW_LATENCY - 1);
  localparam WRITE_SHIFT_DEPTH = 1 << WRITE_SHIFT_ADDRS;

  reg  dqs_q;
  wire dqs_w;

  assign dqs_p = {DDR3_MASKS{~clock}};
  assign dqs_n = {DDR3_MASKS{clock}};

  always @(negedge clock) begin
    if (reset) begin
      dqs_q <= 1'b1;
      dqs_t <= 1'b1;
    end else if (!dqs_w) begin
      dqs_q <= 1'b0;
      dqs_t <= 1'b0;
    end else begin
      {dqs_t, dqs_q} <= {dqs_q, dqs_w};
    end
  end

  wire [WRITE_SHIFT_ADDRS-1:0] wr_dqs_w = wr_lat_q - 2;

  shift_register #(
      .WIDTH(1),
      .DEPTH(16)
  ) wr_srl_inst (
      .clock (clock),
      .wren_i(1'b1),
      .data_i(~dfi_wren_i),
      .addr_i(wr_dqs_w),
      .data_o(dqs_w)
  );


  // -- Write-Data Outputs -- //

  reg clock_270, dt_s;
  wire dt_srl;
  wire [DSB:0] dq_srl;
  wire [SSB:0] dm_srl;
  wire [MSB:0] dq_w;
  wire [QSB:0] dm_w;
  reg [MSB:0] dq_s;
  reg [QSB:0] dm_s;

  assign dq_w = clock_270 ? dq_srl[MSB:0] : dq_srl[DSB:DDR3_WIDTH];
  assign dm_w = clock_270 ? dm_srl[QSB:0] : dm_srl[SSB:DDR3_MASKS];

  always @(negedge clk_ddr) begin
    clock_270 <= clock;
  end

  always @(negedge clk_ddr) begin
    if (reset) begin
      dq_t <= 1'b1;
    end else begin
      {dq_t, dt_s} <= {dt_s, dt_srl};
      {dm_q, dm_s} <= {dm_s, dm_w};
      {dq_q, dq_s} <= {dq_s, dq_w};
    end
  end

  wire [WRITE_SHIFT_ADDRS-1:0] wr_lat_w = wr_lat_q - 1;

  shift_register #(
      .WIDTH(WRITE_SHIFT_WIDTH),
      .DEPTH(WRITE_SHIFT_DEPTH)
  ) dq_srl_inst (
      .clock (clock_270),
      .wren_i(1'b1),
      .data_i({~dfi_wren_i, dfi_mask_i, dfi_data_i}),
      .addr_i(wr_lat_w),
      .data_o({dt_srl, dm_srl, dq_srl})
  );


  // -- Read Data Valid Signals -- //

  wire rd_en_w;

  shift_register #(
      .WIDTH(1),
      .DEPTH(16)
  ) rd_srl_inst (
      .clock (clock),
      .wren_i(1'b1),
      .data_i(dfi_rden_i),
      .addr_i(rd_lat_q),
      .data_o(rd_en_w)
  );

  always @(posedge clock) begin
    if (reset) begin
      valid_q <= 1'b0;
    end else begin
      valid_q <= rd_en_w;
    end
  end


  // -- Data Capture on Read -- //

  reg [MSB:0] data_l, data_n, data_h;

  always @(posedge clock) begin
    data_l <= data_n;
    data_q <= {data_h, data_l};
  end

  always @(posedge ddr3_ck_po) begin
    if (ddr3_dqs_pio) begin
      data_n <= ddr3_dq_io;
    end
  end

  always @(posedge ddr3_ck_no) begin
    if (ddr3_dqs_nio) begin
      data_h <= ddr3_dq_io;
    end
  end


endmodule  // generic_ddr3_phy
