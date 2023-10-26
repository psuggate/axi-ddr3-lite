`timescale 1ns / 100ps
module ddr3_cfg_tb;

  // -- Simulation Settings -- //

  // DDR3 SRAM Timings
  localparam DDR_FREQ_MHZ = 100;
  `include "ddr3_settings.vh"

  localparam DDR_ROW_BITS = 13;
  localparam RSB = DDR_ROW_BITS - 1;

  parameter DDR_COL_BITS = 10;
  localparam CSB = DDR_COL_BITS - 1;

  // Data-path and address settings
  localparam WIDTH = 32;
  localparam MSB = WIDTH - 1;

  localparam MASKS = WIDTH / 8;
  localparam SSB = MASKS - 1;

  localparam REQID = 4;
  localparam ISB = REQID - 1;


  // -- Simulation Data -- //

  initial begin
    $dumpfile("ddr3_cfg_tb.vcd");
    $dumpvars;

    #80000 $finish;  // todo ...
  end


  // -- Globals -- //

  reg osc = 1'b1;
  reg ddr = 1'b1;
  reg rst = 1'b0;

  always #5.0 osc <= ~osc;
  always #2.5 ddr <= ~ddr;

  initial begin
    rst <= 1'b1;
    #200 rst <= 1'b0;
  end


  wire locked, clock, reset;
  wire clk_ddr, clk_ddr_dqs, clk_ref;

  assign #50 locked = 1'b1;
  assign clk_ddr = ddr;
  assign clock = osc;
  assign reset = rst | ~locked;


  // -- DDR3 Configurator for a Memory Controller -- //

  wire ddl_req, ddl_rdy, ddl_ref;
  wire [2:0] ddl_cmd, ddl_ba;
  wire [ISB:0] ddl_tid;
  wire [RSB:0] ddl_adr;

  wire ctl_req, ctl_run, ctl_rdy, ctl_ref;
  wire [2:0] ctl_cmd, ctl_ba;
  wire [RSB:0] ctl_adr;

  wire cfg_req, cfg_run, cfg_rdy, cfg_ref;
  wire [2:0] cfg_cmd, cfg_ba;
  wire [RSB:0] cfg_adr;

  // AXI <-> {FSM, DDL} signals
  wire wr_valid, wr_ready, wr_last;
  wire rd_valid, rd_ready, rd_last;
  wire [SSB:0] wr_mask;
  wire [MSB:0] wr_data, rd_data;

  // DFI <-> PHY
  wire dfi_rst_n, dfi_cke, dfi_cs_n, dfi_ras_n, dfi_cas_n, dfi_we_n;
  wire dfi_odt, dfi_wren, dfi_rden, dfi_valid;
  wire [  2:0] dfi_bank;
  wire [RSB:0] dfi_addr;
  wire [SSB:0] dfi_mask;
  wire [MSB:0] dfi_wdata, dfi_rdata;

  // PHY <-> DDR3
  wire ddr_ck_p, ddr_ck_n;
  wire ddr_rst_n, ddr_cke, ddr_cs_n, ddr_ras_n, ddr_cas_n, ddr_we_n;
  wire ddr_odt;
  wire [2:0] ddr_ba;
  wire [RSB:0] ddr_a;
  wire [1:0] ddr_dm, ddr_dqs_p, ddr_dqs_n;
  wire [15:0] ddr_dq;


  assign dfi_valid = 1'b0;
  assign dfi_mask  = {MASKS{1'b0}};
  assign dfi_wdata = {WIDTH{1'bx}};

  assign wr_valid  = 1'b0;
  assign wr_last   = 1'bx;
  assign rd_ready  = 1'b0;
  assign wr_mask   = {MASKS{1'b0}};
  assign wr_data   = {WIDTH{1'bx}};


  // -- Manage the REFRESH Requests -- //

  reg mem_req;
  reg [2:0] mem_cmd;

  assign ctl_cmd = cfg_run ? mem_cmd : cfg_cmd;
  assign ctl_req = cfg_run ? mem_req : cfg_req;
  assign ctl_ba  = cfg_run ? 'bx : cfg_ba;
  assign ctl_adr = cfg_run ? 'bx : cfg_adr;

  assign cfg_rdy = ctl_rdy;


  always @(posedge clock) begin
    if (reset || !cfg_run) begin
      mem_cmd <= CMD_NOOP;
      mem_req <= 1'b0;
    end else begin
      if (!mem_req && cfg_ref) begin
        mem_cmd <= CMD_REFR;
        mem_req <= 1'b1;
      end else if (mem_req && ctl_rdy) begin
        mem_req <= 1'b0;
        mem_cmd <= CMD_NOOP;
      end
    end
  end


  // -- DDR3 Simulation Model from Micron -- //

  ddr3 ddr3_sdram_inst (
      .rst_n(ddr_rst_n),
      .ck(ddr_ck_p),
      .ck_n(ddr_ck_n),
      .cke(ddr_cke),
      .cs_n(ddr_cs_n),
      .ras_n(ddr_ras_n),
      .cas_n(ddr_cas_n),
      .we_n(ddr_we_n),
      .dm_tdqs(ddr_dm),
      .ba(ddr_ba),
      .addr({1'b0, ddr_a}),
      .dq(ddr_dq),
      .dqs(ddr_dqs_p),
      .dqs_n(ddr_dqs_n),
      .tdqs_n(),
      .odt(ddr_odt)
  );


  // -- DDR3 PHI Interface Modules -- //

  generic_ddr3_phy #(
      .DDR3_WIDTH(16),  // (default)
      .ADDR_BITS(DDR_ROW_BITS)  // default: 14
  ) ddr3_phy_inst (
      .clock  (clock),
      .reset  (reset),
      .clk_ddr(clk_ddr),

      .cfg_valid_i(dfi_valid),
      .cfg_data_i ({16'h0000, 4'h4, 4'h4, 8'h00}),

      .dfi_cke_i (dfi_cke),
      .dfi_rst_ni(dfi_rst_n),
      .dfi_cs_ni (dfi_cs_n),
      .dfi_ras_ni(dfi_ras_n),
      .dfi_cas_ni(dfi_cas_n),
      .dfi_we_ni (dfi_we_n),
      .dfi_odt_i (dfi_odt),
      .dfi_bank_i(dfi_bank),
      .dfi_addr_i(dfi_addr),

      .dfi_wren_i(dfi_wren),
      .dfi_mask_i(dfi_mask),
      .dfi_data_i(dfi_wdata),

      .dfi_rden_i (dfi_rden),
      .dfi_valid_o(dfi_valid),
      .dfi_data_o (dfi_rdata),

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

  // Inserts NOP's between memory-controller commands to satisfy DDR3 timing
  // parameters.
  ddr3_ddl #(
      .DDR_FREQ_MHZ  (DDR_FREQ_MHZ),
      .DDR_ROW_BITS  (DDR_ROW_BITS),
      .DDR_COL_BITS  (DDR_COL_BITS),
      .DFI_DATA_WIDTH(WIDTH)
  ) ddr3_ddl_inst (
      .clock(clock),
      .reset(reset),

      .ddr_cke_i(dfi_cke),
      .ddr_cs_ni(dfi_cs_n),

      .ctl_req_i(ctl_req),
      .ctl_rdy_o(ctl_rdy),
      .ctl_cmd_i(ctl_cmd),
      .ctl_ba_i (ctl_ba),
      .ctl_adr_i(ctl_adr),

      .mem_wvalid_i(wr_valid),
      .mem_wready_o(wr_ready),
      .mem_wlast_i (wr_last),
      .mem_wrmask_i(wr_mask),
      .mem_wrdata_i(wr_data),

      .mem_rvalid_o(rd_valid),
      .mem_rready_i(rd_ready),
      .mem_rlast_o (rd_last),
      .mem_rddata_o(rd_data),

      .dfi_ras_no (dfi_ras_n),
      .dfi_cas_no (dfi_cas_n),
      .dfi_we_no  (dfi_we_n),
      .dfi_bank_o (dfi_bank),
      .dfi_addr_o (dfi_addr),
      .dfi_wren_o (dfi_wren),
      .dfi_mask_o (dfi_mask),
      .dfi_data_o (dfi_wdata),
      .dfi_rden_o (dfi_rden),
      .dfi_valid_i(dfi_valid),
      .dfi_data_i (dfi_rdata)
  );


  // -- Module Under Test -- //

  wire cfg_valid;
  wire [31:0] cfg_data;

  assign cfg_valid = 1'b0;
  assign cfg_data  = 'bx;

  ddr3_cfg #(
      .DDR_FREQ_MHZ(DDR_FREQ_MHZ),
      .DDR_ROW_BITS(DDR_ROW_BITS)
  ) ddr3_cfg_inst (
      .clock(clock),
      .reset(reset),

      .cfg_valid_i(cfg_valid),  // toods ??
      .cfg_ready_o(),
      .cfg_data_i (cfg_data),
      .cfg_data_o (),

      .dfi_rst_no(dfi_rst_n),  // Control these IOB's directly
      .dfi_cke_o (dfi_cke),
      .dfi_cs_no (dfi_cs_n),
      .dfi_odt_o (dfi_odt),

      .ctl_req_o(cfg_req),  // Memory controller signals
      .ctl_run_o(cfg_run),  // When initialisation has completed
      .ctl_rdy_i(cfg_rdy),
      .ctl_cmd_o(cfg_cmd),
      .ctl_ref_o(cfg_ref),
      .ctl_ba_o (cfg_ba),
      .ctl_adr_o(cfg_adr)
  );


endmodule  // ddr3_cfg_tb
