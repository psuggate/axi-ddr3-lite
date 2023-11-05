`timescale 1ns / 100ps
`define __gowin_for_the_win
module axi_ddr3_top (
    // Clock and reset from the dev-board
    aclk,
    aresetn,

    // USB ULPI pins on the dev-board
    ulpi_clock,
    ulpi_reset,
    ulpi_dir,
    ulpi_nxt,
    ulpi_stp,
    ulpi_data,

    // 1Gb DDR3 SDRAM pins
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

  // Data-path widths
  localparam DDR_DQ_WIDTH = 16;
  localparam MSB = DDR_DQ_WIDTH - 1;

  localparam DDR_DM_WIDTH = 2;
  localparam SSB = DDR_DM_WIDTH - 1;

  // Address widths
  localparam DDR_ROW_BITS = 13;
  localparam RSB = DDR_ROW_BITS - 1;

  localparam DDR_COL_BITS = 10;
  localparam CSB = DDR_COL_BITS - 1;


  input aclk;
  input aresetn;

  input ulpi_clock;
  output ulpi_reset;
  input ulpi_dir;
  input ulpi_nxt;
  output ulpi_stp;
  inout ulpi_data;

  output ddr_ck_po;
  output ddr_ck_no;
  output ddr_rst_no;
  output ddr_cke_o;
  output ddr_cs_no;
  output ddr_ras_no;
  output ddr_cas_no;
  output ddr_we_no;
  output ddr_odt_o;
  output [2:0] ddr_ba_o;
  output [RSB:0] ddr_a_o;
  output [SSB:0] ddr_dm_o;
  inout [SSB:0] ddr_dqs_pio;
  inout [SSB:0] ddr_dqs_nio;
  inout [MSB:0] ddr_dq_io;


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
  parameter LOW_LATENCY = 1'b1;  // 0 or 1

  // Size of bursts from memory controller perspective
  parameter DDR_BURSTLEN = 4;

  // AXI4 interconnect properties
  parameter AXI_ID_WIDTH = 4;
  localparam ISB = AXI_ID_WIDTH - 1;


  // -- USB ULPI Bulk transfer endpoint (IN & OUT) -- //

  wire ulpi_data_t;
  wire [7:0] ulpi_data_o;

  assign ulpi_rst  = usb_rst_n;
  assign usb_clk   = ~ulpi_clk;
  assign ulpi_data = ulpi_data_t ? {8{1'bz}} : ulpi_data_o;

  wire s_tvalid, s_tready, s_tlast;
  wire [7:0] s_tdata;

  wire m_tvalid, m_tready, m_tlast;
  wire [7:0] m_tdata;

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

      .aclk(axi_clk),
      .aresetn(reset_n),

      .s_axis_tvalid_i(s_tvalid),
      .s_axis_tready_o(s_tready),
      .s_axis_tlast_i (s_tlast),
      .s_axis_tdata_i (s_tdata),

      .m_axis_tvalid_o(m_tvalid),
      .m_axis_tready_i(m_tready),
      .m_axis_tlast_o (m_tlast),
      .m_axis_tdata_o (m_tdata)
  );


  // -- DDR3 PHY -- //

`ifdef __gowin_for_the_win

  // GoWin Global System Reset signal tree.
  GSR GSR ();

  gw2a_ddr3_phy #(
      .DDR3_WIDTH(16),  // (default)
      .ADDR_BITS(DDR_ROW_BITS),  // default: 14
      .SOURCE_CLOCK(2'b01),  // (default)
      .CAPTURE_DELAY(3'h2)  // (default)
  ) u_phy (
      .clock  (clock),
      .reset  (reset),
      .clk_ddr(clk_ddr),

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

      .ddr_ck_po(ddr_ck_p),
      .ddr_ck_no(ddr_ck_n),
      .ddr_rst_no(ddr_rst_n),
      .ddr_cke_o(ddr_cke),
      .ddr_cs_no(ddr_cs_n),
      .ddr_ras_no(ddr_ras_n),
      .ddr_cas_no(ddr_cas_n),
      .ddr_we_no(ddr_we_n),
      .ddr_odt_o(ddr_odt),
      .ddr_ba_o(ddr_ba),
      .ddr_a_o(ddr_a),
      .ddr_dm_o(ddr_dm),
      .ddr_dqs_pio(ddr_dqs_p),
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
      .clk_ddr(clk_ddr),

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

      .ddr3_ck_po(ddr_ck_p),
      .ddr3_ck_no(ddr_ck_n),
      .ddr3_cke_o(ddr_cke),
      .ddr3_rst_no(ddr_rst_n),
      .ddr3_cs_no(ddr_cs_n),
      .ddr3_ras_no(ddr_ras_n),
      .ddr3_cas_no(ddr_cas_n),
      .ddr3_we_no(ddr_we_n),
      .ddr3_odt_o(ddr_odt),
      .ddr3_ba_o(ddr_ba),
      .ddr3_a_o(ddr_a),
      .ddr3_dm_o(ddr_dm),
      .ddr3_dqs_pio(ddr_dqs_p),
      .ddr3_dqs_nio(ddr_dqs_n),
      .ddr3_dq_io(ddr_dq)
  );

`endif


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

      .byp_arvalid_i(abvalid),  // [optional] fast-read port
      .byp_arready_o(abready),
      .byp_araddr_i(byaddr),
      .byp_arid_i(byid),
      .byp_arlen_i(bylen),
      .byp_arburst_i(byburst),

      .byp_rready_i(dbready),
      .byp_rvalid_o(dbvalid),
      .byp_rlast_o(dblast),
      .byp_rresp_o(dbresp),
      .byp_rid_o(dbid),
      .byp_rdata_o(bdata),

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


endmodule  // axi_ddr3_top
