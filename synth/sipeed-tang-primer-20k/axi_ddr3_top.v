`timescale 1ns / 100ps
// `define __gowin_for_the_win
module axi_ddr3_top (
    // Clock and reset from the dev-board
    clk_26,
    rst_n,

    // USB ULPI pins on the dev-board
    ulpi_clk,
    ulpi_rst,
    ulpi_dir,
    ulpi_nxt,
    ulpi_stp,
    ulpi_data,

    // 1Gb DDR3 SDRAM pins
    ddr_ck,
    ddr_ck_n,
    ddr_cke,
    ddr_rst_n,
    ddr_cs,
    ddr_ras,
    ddr_cas,
    ddr_we,
    ddr_odt,
    ddr_bank,
    ddr_addr,
    ddr_dm,
    ddr_dqs,
    ddr_dqs_n,
    ddr_dq
);

  localparam WIDTH = 32;
  localparam MSB = WIDTH - 1;
  localparam MASKS = WIDTH / 8;
  localparam SSB = MASKS - 1;

  localparam ADDRS = 24;
  localparam ASB = ADDRS - 1;

  localparam REQID = 4;
  localparam ISB = REQID - 1;

  // Data-path widths
  localparam DDR_DQ_WIDTH = 16;
  localparam DSB = DDR_DQ_WIDTH - 1;

  localparam DDR_DM_WIDTH = 2;
  localparam QSB = DDR_DM_WIDTH - 1;

  // Address widths
  localparam DDR_ROW_BITS = 13;
  localparam RSB = DDR_ROW_BITS - 1;

  localparam DDR_COL_BITS = 10;
  localparam CSB = DDR_COL_BITS - 1;

  localparam FPGA_VENDOR = "gowin";
  localparam FPGA_FAMILY = "gw2a";
  localparam [63:0] SERIAL_NUMBER = "GULP0123";

  localparam HIGH_SPEED = 1'b1;
  localparam CHANNEL_IN_ENABLE = 1'b1;
  localparam CHANNEL_OUT_ENABLE = 1'b1;
  localparam PACKET_MODE = 1'b0;


  input clk_26;
  input rst_n;

  input ulpi_clk;
  output ulpi_rst;
  input ulpi_dir;
  input ulpi_nxt;
  output ulpi_stp;
  inout [7:0] ulpi_data;

  output ddr_ck;
  output ddr_ck_n;
  output ddr_rst_n;
  output ddr_cke;
  output ddr_cs;
  output ddr_ras;
  output ddr_cas;
  output ddr_we;
  output ddr_odt;
  output [2:0] ddr_bank;
  output [RSB:0] ddr_addr;
  output [QSB:0] ddr_dm;
  inout [QSB:0] ddr_dqs;
  inout [QSB:0] ddr_dqs_n;
  inout [DSB:0] ddr_dq;


  // -- Constants -- //

  // Settings for DLL=off mode
  parameter DDR_FREQ_MHZ = 100;
  parameter DDR_CL = 6;
  parameter DDR_CWL = 6;

`ifdef __gowin_for_the_win
  localparam PHY_WR_DELAY = 3;
  localparam PHY_RD_DELAY = 1;
  localparam WR_PREFETCH = 1'b1;
`else
  localparam PHY_WR_DELAY = 1;
  localparam PHY_RD_DELAY = 1;
  localparam WR_PREFETCH = 1'b0;
`endif

  // Trims an additional clock-cycle of latency, if '1'
  parameter LOW_LATENCY = 1'b0;  // 0 or 1

  // Size of bursts from memory controller perspective
  parameter DDR_BURSTLEN = 4;


  wire clock, rst_n, reset;
  wire axi_clk, ddr_clk, usb_clk, usb_rst_n;

  assign reset   = ~rst_n;
  assign axi_clk = clock;

  // So 27.0 MHz divided by 9, then x40 = 120 MHz.
  gw2a_rpll #(
      .FCLKIN("27"),
      .IDIV_SEL(8),  // ~=  9
      .FBDIV_SEL(39),  // ~= 40
      .ODIV_SEL(8)
  ) axis_rpll_inst (
      .clkout(clock),    // 120 MHz
      .clockd(),
      // .clockd(ddr_clk),
      .lock  (rst_n),
      .clkin (clk_26)
  );

  // So 27.0 MHz divided by 9, then x40 = 120 MHz.
  gw2a_rpll #(
      .FCLKIN("27"),
      .IDIV_SEL(3),  // ~=  4
      .FBDIV_SEL(58),  // ~= 59
      .ODIV_SEL(2)
  ) ddr3_rpll_inst (
      .clkout(ddr_clk),    // 120 MHz
      .clockd(),
      .lock  (rst_n),
      .clkin (clk_26)
  );


  wire s_tvalid, s_tready, s_tlast;
  wire [7:0] s_tdata;

  wire m_tvalid, m_tready, m_tlast;
  wire [7:0] m_tdata;

  // AXI4 Signals to/from the Memory Controller
  wire awvalid, wvalid, wlast, bready, arvalid, rready;
  wire awready, wready, bvalid, arready, rvalid, rlast;
  wire [ISB:0] awid, arid, bid, rid;
  wire [7:0] awlen, arlen;
  wire [1:0] awburst, arburst;
  wire [ASB:0] awaddr, araddr;
  wire [SSB:0] wstrb;
  wire [1:0] bresp, rresp;
  wire [MSB:0] rdata, wdata;

  // DFI <-> PHY
  wire dfi_rst_n, dfi_cke, dfi_cs_n, dfi_ras_n, dfi_cas_n, dfi_we_n;
  wire dfi_odt, dfi_wstb, dfi_wren, dfi_rden, dfi_valid, dfi_last;
  wire [  2:0] dfi_bank;
  wire [RSB:0] dfi_addr;
  wire [SSB:0] dfi_mask;
  wire [MSB:0] dfi_wdata, dfi_rdata;


  // -- Controls the DDR3 via USB -- //

  axis_ddr3_ctrl axdr_ctrl_inst (
      .clock(clock),
      .reset(reset),

      .s_valid_i(s_tvalid),
      .s_ready_o(s_tready),
      .s_last_i (s_tlast),
      .s_data_i (s_tdata),

      .m_valid_o(m_tvalid),
      .m_ready_i(m_tready),
      .m_last_o (m_tlast),
      .m_data_o (m_tdata),

      .awvalid_o(awvalid),
      .awready_i(awready),
      .awburst_o(awburst),
      .awlen_o(awlen),
      .awid_o(awid),
      .awaddr_o(awaddr),

      .wvalid_o(wvalid),
      .wready_i(wready),
      .wlast_o (wlast),
      .wstrb_o (wstrb),
      .wdata_o (wdata),

      .bvalid_i(bvalid),
      .bready_o(bready),
      .bid_i(bid),
      .bresp_i(bresp),

      .arvalid_o(arvalid),
      .arready_i(arready),
      .arburst_o(arburst),
      .arlen_o(arlen),
      .arid_o(arid),
      .araddr_o(araddr),

      .rvalid_i(rvalid),
      .rready_o(rready),
      .rlast_i(rlast),
      .rid_i(rid),
      .rresp_i(rresp),
      .rdata_i(rdata)
  );


  // -- USB ULPI Bulk transfer endpoint (IN & OUT) -- //

  wire ulpi_data_t;
  wire [7:0] ulpi_data_o;

  assign ulpi_rst  = usb_rst_n;
  assign usb_clk   = ~ulpi_clk;
  assign ulpi_data = ulpi_data_t ? {8{1'bz}} : ulpi_data_o;

  ulpi_bulk_axis #(
      .FPGA_VENDOR(FPGA_VENDOR),
      .FPGA_FAMILY(FPGA_FAMILY),
      .VENDOR_ID(16'hF4CE),
      .PRODUCT_ID(16'h0003),
      .HIGH_SPEED(HIGH_SPEED),
      .SERIAL_NUMBER(SERIAL_NUMBER),
      .CHANNEL_IN_ENABLE(CHANNEL_IN_ENABLE),
      .CHANNEL_OUT_ENABLE(CHANNEL_OUT_ENABLE),
      .PACKET_MODE(PACKET_MODE)
  ) ulpi_bulk_axis_inst (
      .ulpi_clock_i(usb_clk),
      .ulpi_reset_o(usb_rst_n),

      .ulpi_dir_i (ulpi_dir),
      .ulpi_nxt_i (ulpi_nxt),
      .ulpi_stp_o (ulpi_stp),
      .ulpi_data_t(ulpi_data_t),
      .ulpi_data_i(ulpi_data),
      .ulpi_data_o(ulpi_data_o),

      .aclk(clock),
      .aresetn(rst_n),

      .s_axis_tvalid_i(m_tvalid),
      .s_axis_tready_o(m_tready),
      .s_axis_tlast_i (m_tlast),
      .s_axis_tdata_i (m_tdata),

      .m_axis_tvalid_o(s_tvalid),
      .m_axis_tready_i(s_tready),
      .m_axis_tlast_o (s_tlast),
      .m_axis_tdata_o (s_tdata)
  );


  //
  //  DDR Core Under New Test
  ///

  axi_ddr3_lite #(
      .DDR_FREQ_MHZ (DDR_FREQ_MHZ),
      .DDR_ROW_BITS (DDR_ROW_BITS),
      .DDR_COL_BITS (DDR_COL_BITS),
      .DDR_DQ_WIDTH (DDR_DQ_WIDTH),
      .PHY_WR_DELAY (PHY_WR_DELAY),
      .PHY_RD_DELAY (PHY_RD_DELAY),
      .WR_PREFETCH  (WR_PREFETCH),
      .LOW_LATENCY  (LOW_LATENCY),
      .AXI_ID_WIDTH (REQID),
      .MEM_ID_WIDTH (REQID),
      .BYPASS_ENABLE(0)
  ) ddr_core_inst (
      .clock(clock),  // system clock
      .reset(reset),  // synchronous reset

      .axi_awvalid_i(awvalid),
      .axi_awready_o(awready),
      .axi_awaddr_i(awaddr),
      .axi_awid_i(awid),
      .axi_awlen_i(awlen),
      .axi_awburst_i(awburst),

      .axi_wvalid_i(wvalid),
      .axi_wready_o(wready),
      .axi_wlast_i (wlast),
      .axi_wstrb_i (wstrb),
      .axi_wdata_i (wdata),

      .axi_bvalid_o(bvalid),
      .axi_bready_i(bready),
      .axi_bresp_o(bresp),
      .axi_bid_o(bid),

      .axi_arvalid_i(arvalid),
      .axi_arready_o(arready),
      .axi_araddr_i(araddr),
      .axi_arid_i(arid),
      .axi_arlen_i(arlen),
      .axi_arburst_i(arburst),

      .axi_rvalid_o(rvalid),
      .axi_rready_i(rready),
      .axi_rlast_o(rlast),
      .axi_rresp_o(rresp),
      .axi_rid_o(rid),
      .axi_rdata_o(rdata),

      .byp_arvalid_i(1'b0),  // [optional] fast-read port
      .byp_arready_o(),
      .byp_araddr_i('bx),
      .byp_arid_i('bx),
      .byp_arlen_i('bx),
      .byp_arburst_i('bx),

      .byp_rready_i(1'b0),
      .byp_rvalid_o(),
      .byp_rlast_o(),
      .byp_rresp_o(),
      .byp_rid_o(),
      .byp_rdata_o(),

      .dfi_rst_no(dfi_rst_n),
      .dfi_cke_o (dfi_cke),
      .dfi_cs_no (dfi_cs_n),
      .dfi_ras_no(dfi_ras_n),
      .dfi_cas_no(dfi_cas_n),
      .dfi_we_no (dfi_we_n),
      .dfi_odt_o (dfi_odt),
      .dfi_bank_o(dfi_bank),
      .dfi_addr_o(dfi_addr),

      .dfi_wstb_o(dfi_wstb),
      .dfi_wren_o(dfi_wren),
      .dfi_mask_o(dfi_mask),
      .dfi_data_o(dfi_wdata),

      .dfi_rden_o(dfi_rden),
      .dfi_rvld_i(dfi_valid),
      .dfi_last_i(dfi_last),
      .dfi_data_i(dfi_rdata)
  );


  // -- DDR3 PHY -- //

`ifdef __gowin_for_the_win

  gw2a_ddr3_phy #(
      .DDR3_WIDTH(16),  // (default)
      .ADDR_BITS(DDR_ROW_BITS),  // default: 14
      .SOURCE_CLOCK(2'b01),  // (default)
      .CAPTURE_DELAY(3'h2)  // (default)
  ) u_phy (
      .clock  (clock),
      .reset  (reset),
      .clk_ddr(ddr_clk),

      .dfi_rst_ni(dfi_rst_n),
      .dfi_cke_i (dfi_cke),
      .dfi_cs_ni (dfi_cs_n),
      .dfi_ras_ni(dfi_ras_n),
      .dfi_cas_ni(dfi_cas_n),
      .dfi_we_ni (dfi_we_n),
      .dfi_odt_i (dfi_odt),
      .dfi_bank_i(dfi_bank),
      .dfi_addr_i(dfi_addr),

      .dfi_wstb_i(dfi_wstb),
      .dfi_wren_i(dfi_wren),
      .dfi_mask_i(dfi_mask),
      .dfi_data_i(dfi_wdata),

      .dfi_rden_i(dfi_rden),
      .dfi_rvld_o(dfi_valid),
      .dfi_last_o(dfi_last),
      .dfi_data_o(dfi_rdata),

      .ddr_ck_po(ddr_ck),
      .ddr_ck_no(ddr_ck_n),
      .ddr_rst_no(ddr_rst_n),
      .ddr_cke_o(ddr_cke),
      .ddr_cs_no(ddr_cs),
      .ddr_ras_no(ddr_ras),
      .ddr_cas_no(ddr_cas),
      .ddr_we_no(ddr_we),
      .ddr_odt_o(ddr_odt),
      .ddr_ba_o(ddr_bank),
      .ddr_a_o(ddr_addr),
      .ddr_dm_o(ddr_dm),
      .ddr_dqs_pio(ddr_dqs),
      .ddr_dqs_nio(ddr_dqs_n),
      .ddr_dq_io(ddr_dq)
  );

`else

  // Generic PHY -- that probably won't synthesise correctly, due to how the
  // (read-)data is registered ...
  generic_ddr3_phy #(
      .DDR3_WIDTH(16),  // (default)
      .ADDR_BITS(DDR_ROW_BITS)  // default: 14
  ) ddr3_phy_inst (
      .clock  (clock),
      .reset  (reset),
      .clk_ddr(ddr_clk),

      .dfi_rst_ni(dfi_rst_n),
      .dfi_cke_i (dfi_cke),
      .dfi_cs_ni (dfi_cs_n),
      .dfi_ras_ni(dfi_ras_n),
      .dfi_cas_ni(dfi_cas_n),
      .dfi_we_ni (dfi_we_n),
      .dfi_odt_i (dfi_odt),
      .dfi_bank_i(dfi_bank),
      .dfi_addr_i(dfi_addr),

      .dfi_wstb_i(dfi_wstb),
      .dfi_wren_i(dfi_wren),
      .dfi_mask_i(dfi_mask),
      .dfi_data_i(dfi_wdata),

      .dfi_rden_i(dfi_rden),
      .dfi_rvld_o(dfi_valid),
      .dfi_last_o(dfi_last),
      .dfi_data_o(dfi_rdata),

      .ddr3_ck_po(ddr_ck),
      .ddr3_ck_no(ddr_ck_n),
      .ddr3_cke_o(ddr_cke),
      .ddr3_rst_no(ddr_rst_n),
      .ddr3_cs_no(ddr_cs),
      .ddr3_ras_no(ddr_ras),
      .ddr3_cas_no(ddr_cas),
      .ddr3_we_no(ddr_we),
      .ddr3_odt_o(ddr_odt),
      .ddr3_ba_o(ddr_bank),
      .ddr3_a_o(ddr_addr),
      .ddr3_dm_o(ddr_dm),
      .ddr3_dqs_pio(ddr_dqs),
      .ddr3_dqs_nio(ddr_dqs_n),
      .ddr3_dq_io(ddr_dq)
  );

`endif


endmodule  // axi_ddr3_top
