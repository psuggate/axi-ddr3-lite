`timescale 1ns / 100ps
/**
 * Converts simple memory-controller commands into DFI commands.
 * 
 * Notes:
 *  - assumes that the AXI4 interface converts write-data into 128-bit chunks,
 *    (written as 4x 32-bit sequential transfers) padding as required;
 *  - read data will also be a (continuous) stream of 128-bit chunks, so the
 *    AXI4 interface will have to drop any (unwanted) trailing data, if not
 *    required;
 *  - assumes that the memory controller and the AXI4 bus are within the same
 *    clock-domain;
 * 
 * Copyright 2023, Patrick Suggate.
 * 
 */
module gw2a_ddr3_phy (
    clock,
    reset,

    clk_ddr,

    dfi_cke_i,
    dfi_rst_ni,
    dfi_cs_ni,
    dfi_ras_ni,
    dfi_cas_ni,
    dfi_we_ni,
    dfi_odt_i,
    dfi_bank_i,
    dfi_addr_i,

    dfi_wstb_i,
    dfi_wren_i,
    dfi_mask_i,
    dfi_data_i,

    dfi_rden_i,
    dfi_rvld_o,
    dfi_last_o,
    dfi_data_o,

    ddr_ck_po,
    ddr_ck_no,
    ddr_cke_o,
    ddr_rst_no,
    ddr_cs_no,
    ddr_ras_no,
    ddr_cas_no,
    ddr_we_no,
    ddr_odt_o,
    ddr_ba_o,
    ddr_a_o,
    ddr_dm_o,
    ddr_dqs_pio,
    ddr_dqs_nio,
    ddr_dq_io
);

  parameter DDR3_WIDTH = 16;
  parameter DDR3_MASKS = DDR3_WIDTH / 8;

  localparam MSB = DDR3_WIDTH - 1;
  localparam QSB = DDR3_MASKS - 1;

  localparam DSB = DDR3_WIDTH + MSB;
  localparam SSB = DDR3_MASKS + QSB;

  parameter ADDR_BITS = 14;
  localparam ASB = ADDR_BITS - 1;


  input clock;
  input reset;

  input clk_ddr;  // Same phase, but twice freq of 'clock'

  input dfi_cke_i;
  input dfi_rst_ni;
  input dfi_cs_ni;
  input dfi_ras_ni;
  input dfi_cas_ni;
  input dfi_we_ni;
  input dfi_odt_i;

  input [2:0] dfi_bank_i;
  input [ASB:0] dfi_addr_i;

  input dfi_wstb_i;
  input dfi_wren_i;
  input [SSB:0] dfi_mask_i;
  input [DSB:0] dfi_data_i;

  input dfi_rden_i;
  output dfi_rvld_o;
  output dfi_last_o;
  output [DSB:0] dfi_data_o;

  output ddr_ck_po;
  output ddr_ck_no;
  output ddr_cke_o;
  output ddr_rst_no;
  output ddr_cs_no;
  output ddr_ras_no;
  output ddr_cas_no;
  output ddr_we_no;
  output ddr_odt_o;
  output [2:0] ddr_ba_o;
  output [ASB:0] ddr_a_o;
  output [QSB:0] ddr_dm_o;
  inout [QSB:0] ddr_dqs_pio;
  inout [QSB:0] ddr_dqs_nio;
  inout [MSB:0] ddr_dq_io;


  wire [QSB:0] dqs_t, dqs_s, dm_w;
  wire [QSB:0] dqs_p, dqs_n;
  wire [MSB:0] dq_t, dq_w;

  reg cke_q, rst_nq, cs_nq;
  reg ras_nq, cas_nq, we_nq, odt_q;
  reg [2:0] ba_q;

  reg delay_q, valid_q, last_q;
  reg [ASB:0] addr_q;
  reg [DSB:0] data_q;


  // -- DFI Read-Data Signal Assignments -- //

  assign dfi_rvld_o = valid_q;
  assign dfi_last_o = last_q;


  // -- DDR3 Signal Assignments -- //

  assign ddr_ck_po  = ~clock;
  assign ddr_ck_no  = clock;

  assign ddr_cke_o  = cke_q;
  assign ddr_rst_no = rst_nq;
  assign ddr_cs_no  = cs_nq;
  assign ddr_ras_no = ras_nq;
  assign ddr_cas_no = cas_nq;
  assign ddr_we_no  = we_nq;
  assign ddr_odt_o  = odt_q;
  assign ddr_ba_o   = ba_q;
  assign ddr_a_o    = addr_q;


  // -- IOB DDR Register Settings -- //

  localparam CLOCK_POLARITY = 1'b0;
  localparam DATA_ODDR_INIT = 1'b0;
  localparam DQSX_ODDR_INIT = 1'b1;


  // -- DDR3 Command Signals -- //

  always @(posedge clock) begin
    if (reset) begin
      cke_q  <= 1'b0;
      rst_nq <= 1'b0;
      cs_nq  <= 1'b1;
      ras_nq <= 1'b1;
      cas_nq <= 1'b1;
      we_nq  <= 1'b1;
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


  // -- Read Data Valid Signals -- //

  always @(posedge clock) begin
    if (reset) begin
      delay_q <= 1'b0;
      valid_q <= 1'b0;
      last_q  <= 1'b0;
    end else begin
      {valid_q, delay_q} <= {delay_q, dfi_rden_i};
      last_q <= delay_q & ~dfi_rden_i;
    end
  end


  // -- DDR3 Data Path IOBs -- //

  localparam SHIFT = 3'b100;

  generate
    for (genvar ii = 0; ii < DDR3_WIDTH; ii++) begin : gen_dq_iobs

      gw2a_ddr_iob #(
          .SHIFT(SHIFT)
      ) u_gw2a_dq_iob (
          .PCLK(clock),
          .FCLK(~clk_ddr),
          .RESET(reset),
          .OEN(~dfi_wren_i),
          .D0(dfi_data_i[ii]),
          .D1(dfi_data_i[DDR3_WIDTH + ii]),
          .Q0(dfi_data_o[ii]),
          .Q1(dfi_data_o[DDR3_WIDTH + ii]),
          .IO(ddr_dq_io[ii])
      );

    end
  endgenerate


  // -- Write-Data Masks Outputs -- //

  generate
    for (genvar ii = 0; ii < DDR3_MASKS; ii++) begin : gen_dm_iobs

      gw2a_ddr_iob #(
          .SHIFT(SHIFT)
      ) u_gw2a_dm_iob (
          .PCLK(clock),
          .FCLK(~clk_ddr),
          .RESET(reset),
          .OEN(~dfi_wren_i),
          .D0(~dfi_mask_i[ii]),
          .D1(~dfi_mask_i[DDR3_MASKS + ii]),
          .Q0(),
          .Q1(),
          .IO(ddr_dm_o[ii])
      );

    end
  endgenerate


  // -- Read- & Write- Data Strobes -- //

`ifdef __fancy_salad
  // todo: WIP
  // todo: TLVDS !?
  wire dqs_w;

  assign dqs_w = ~dfi_wstb_i & ~dfi_wren_i;

  generate
    for (genvar ii = 0; ii < DDR3_MASKS; ii++) begin : gen_dqs_iobs

      gw2a_ddr_iob #(
          .SHIFT(SHIFT)
      ) u_gw2a_dqs_p_iob (
          .PCLK(clock),
          .FCLK(clk_ddr),
          .RESET(reset),
          .OEN(dqs_w),
          .D0(1'b1),
          .D1(1'b0),
          .Q0(),
          .Q1(),
          .IO(ddr_dqs_pio[ii])
      );

    end
  endgenerate

`else
  // note: Does not synthesise !!
  reg dqs_q;
  wire dqs_w;
  wire [QSB:0] dqs_pi;

  assign dqs_w = ~dfi_wstb_i & ~dfi_wren_i;

  always @(posedge clock) begin
    if (reset) begin
      dqs_q <= 1'b1;
    end else begin
      dqs_q <= dqs_w;
    end
  end

  ODDR #(
      .TXCLK_POL(CLOCK_POLARITY),
      .INIT(DQSX_ODDR_INIT)
  ) dqs_p_oddr_inst[QSB:0] (
      .CLK(~clock),
      .TX (dqs_w),
      .D0 (1'b1),
      .D1 (1'b0),
      .Q0 (dqs_p),
      .Q1 (dqs_t)
  );

  TLVDS_IOBUF dqs_iobuf_inst[QSB:0] (
      .I  (dqs_p),
      .OEN(dqs_t),
      .O  (dqs_pi),
      .IO (ddr_dqs_pio),
      .IOB(ddr_dqs_nio)
  );

`endif


endmodule  // gw2a_ddr3_phy
