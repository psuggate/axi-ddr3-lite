`timescale 1ns / 100ps
module axi_ddr3_lite (
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

    dfi_rst_no,
    dfi_cke_o,
    dfi_cs_no,
    dfi_ras_no,
    dfi_cas_no,
    dfi_we_no,
    dfi_odt_o,
    dfi_bank_o,
    dfi_addr_o,
    dfi_wren_o,
    dfi_mask_o,
    dfi_data_o,
    dfi_rden_o,
    dfi_valid_i,
    dfi_data_i
);

  // Settings for DLL=off mode
  parameter DDR_FREQ_MHZ = 100;
  parameter DDR_CL = 6;
  parameter DDR_CWL = 6;
  parameter DDR_DLL_OFF = 1;

  // Size of bursts from memory controller perspective
  parameter PHY_BURSTLEN = 4;

  // Address widths
  parameter DDR_ROW_BITS = 15;
  localparam RSB = DDR_ROW_BITS - 1;
  parameter DDR_COL_BITS = 10;
  localparam CSB = DDR_COL_BITS - 1;

  localparam ASB = DDR_ROW_BITS + DDR_COL_BITS - 1;  // todo ...

  // Data-path widths
  parameter PHY_DAT_BITS = DDR_DQ_WIDTH * 2;
  localparam MSB = PHY_DAT_BITS - 1;
  parameter PHY_STB_BITS = DDR_DM_WIDTH * 2;
  localparam SSB = PHY_STB_BITS - 1;

  // AXI4 interconnect properties
  parameter AXI_ID_WIDTH = 4;
  localparam ISB = AXI_ID_WIDTH - 1;


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

  output dfi_rst_no;
  output dfi_ck_po;
  output dfi_ck_no;
  output dfi_cke_o;
  output dfi_cs_no;
  output dfi_ras_no;
  output dfi_cas_no;
  output dfi_we_no;
  output dfi_odt_o;
  output [2:0] dfi_bank_o;
  output [RSB:0] dfi_addr_o;
  output dfi_wren_o;
  output [SSB:0] dfi_mask_o;
  output [MSB:0] dfi_data_o;
  output dfi_rden_o;
  input dfi_valid_i;
  input [MSB:0] dfi_data_i;


  wire [MSB:0] dfi_wdata, dfi_rdata, mem_rdata;

  // AXI <-> FSM signals
  wire fsm_fetch, fsm_store, fsm_accept, fsm_error;
  wire [ASB:0] fsm_maddr;

  // AXI <-> {FSM, DDL} signals
  wire wr_valid, wr_ready, wr_last;
  wire rd_valid, rd_ready, rd_last;
  wire [SSB:0] wr_mask;
  wire [MSB:0] wr_data, rd_data;


  assign dfi_data_o = dfi_wdata;

  assign dfi_rdata  = dfi_data_i;


  // -- AXI Requests to DDR3 Requests -- //

  ddr3_axi_ctrl #(
      .DDR_FREQ_MHZ(DDR_FREQ_MHZ)
  ) ddr3_axi_ctrl_inst (
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

      .mem_store_o (fsm_store),
      .mem_fetch_o (fsm_fetch),
      .mem_accept_i(fsm_accept),
      .mem_error_i (fsm_error),
      .mem_req_id_o(fsm_reqid),
      .mem_addr_o  (fsm_maddr),

      .mem_valid_o (wr_valid),
      .mem_ready_i (wr_ready),
      .mem_last_o  (wr_last),
      .mem_wrmask_o(wr_mask),
      .mem_wrdata_o(wr_data),

      .mem_valid_i(rd_valid),
      .mem_ready_o(rd_ready),
      .mem_last_i(rd_last),
      .mem_resp_id_i(rd_respi),
      .mem_rddata_i(rd_data)
  );


  // -- DDR3 Memory Controller -- //

  wire ddl_req, ddl_rdy, ddl_ref;
  wire [2:0] ddl_cmd, ddl_ba;
  wire [ISB:0] ddl_tid;
  wire [RSB:0] ddl_adr;

  ddr3_fsm #(
      .DDR_BURSTLEN(DDR_BURSTLEN),
      .DDR_ROW_BITS(DDR_ROW_BITS),
      .DDR_COL_BITS(DDR_COL_BITS),
      .DDR_FREQ_MHZ(DDR_FREQ_MHZ)
  ) ddr3_fsm_inst (
      .clock(clock),
      .reset(reset),

      .mem_wrreq_i(fsm_wrreq),
      .mem_wrack_o(fsm_wrack),
      .mem_wrerr_o(fsm_wrerr),
      .mem_wrtid_i(fsm_wrtid),
      .mem_wradr_i(fsm_wradr),

      .mem_rdreq_i(fsm_rdreq),
      .mem_rdack_o(fsm_rdack),
      .mem_rderr_o(fsm_rderr),
      .mem_rdtid_i(fsm_rdtid),
      .mem_rdadr_i(fsm_rdadr),

      .ddl_req_o(ddl_req),
      .ddl_rdy_i(ddl_rdy),
      .ddl_ref_i(ddl_ref),
      .ddl_cmd_o(ddl_cmd),
      .ddl_tid_o(ddl_tid),
      .ddl_ba_o (ddl_ba),
      .ddl_adr_o(ddl_adr)
  );


  // -- Generic DDR3 to PHY Interface -- //

  ddr3_ddl #(
      .DDR_BURSTLEN(DDR_BURSTLEN),
      .DDR_ROW_BITS(DDR_ROW_BITS),
      .DDR_COL_BITS(DDR_COL_BITS),
      .DDR_FREQ_MHZ(DDR_FREQ_MHZ),
      .DDR_DQ_WIDTH(DDR_DQ_WIDTH),
      .DDR_DM_WIDTH(DDR_DM_WIDTH)
  ) ddr3_ddl_inst (
      .clock(clock),
      .reset(reset),

      .ctl_req_o(ddl_req),
      .ctl_rdy_i(ddl_rdy),
      .ctl_ref_i(ddl_ref),
      .ctl_cmd_o(ddl_cmd),
      .ctl_tid_o(ddl_tid),
      .ctl_ba_o (ddl_ba),
      .ctl_adr_o(ddl_adr),

      .mem_wvalid_i(wr_valid),
      .mem_wready_o(wr_ready),
      .mem_wlast_i (wr_last),
      .mem_wrmask_o(wr_mask),
      .mem_wrdata_o(wr_data),

      .mem_rvalid_i(rd_valid),
      .mem_rready_o(rd_ready),
      .mem_rlast_i (rd_last),
      .mem_respid_i(rd_respi),
      .mem_rddata_i(rd_data),

      .dfi_rst_no (dfi_rst_no),
      .dfi_cke_o  (dfi_cke_o),
      .dfi_cs_no  (dfi_cs_no),
      .dfi_ras_no (dfi_ras_no),
      .dfi_cas_no (dfi_cas_no),
      .dfi_we_no  (dfi_we_no),
      .dfi_odt_o  (dfi_odt_o),
      .dfi_bank_o (dfi_bank_o),
      .dfi_addr_o (dfi_addr_o),
      .dfi_wren_o (dfi_wren_o),
      .dfi_mask_o (dfi_mask_o),
      .dfi_data_o (dfi_data_o),
      .dfi_rden_o (dfi_rden_o),
      .dfi_valid_i(dfi_valid_i),
      .dfi_data_i (dfi_data_o)
  );


endmodule  // axi_ddr3_lite
