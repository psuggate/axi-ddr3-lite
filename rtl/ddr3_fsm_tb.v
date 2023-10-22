`timescale 1ns / 100ps
module ddr3_fsm_tb;

  // -- Simulation Settings -- //

  // DDR3 SRAM Timings
  localparam DDR_FREQ_MHZ = 100;
  `include "ddr3_settings.vh"

  localparam DDR_ROW_BITS = 13;
  localparam RSB = DDR_ROW_BITS - 1;
  localparam DDR_COL_BITS = 10;
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
    $dumpfile("ddr3_fsm_tb.vcd");
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


  wire fsm_wrreq, fsm_wrack, fsm_wrerr;
  wire fsm_rdreq, fsm_rdack, fsm_rderr;
  wire [ISB:0] fsm_wrtid, fsm_rdtid;
  wire [RSB:0] fsm_wradr, fsm_rdadr;


  // -- DDR3 Memory Controller -- //

  wire ddl_req, ddl_rdy, ddl_ref;
  wire [2:0] ddl_cmd, ddl_ba;
  wire [ISB:0] ddl_tid;
  wire [RSB:0] ddl_adr;

  wire cfg_req, cfg_run, cfg_rdy, cfg_ref;
  wire [2:0] cfg_cmd, cfg_ba;
  wire [RSB:0] cfg_adr;

  ddr3_fsm #(
      .DDR_ROW_BITS(DDR_ROW_BITS),
      .DDR_COL_BITS(DDR_COL_BITS),
      .DDR_FREQ_MHZ(DDR_FREQ_MHZ)
  ) ddr3_fsm_inst (
      .clock(clock),
      .reset(reset),

      .mem_wrreq_i(fsm_wrreq),  // Bus -> Controller requests
      .mem_wrack_o(fsm_wrack),
      .mem_wrerr_o(fsm_wrerr),
      .mem_wrtid_i(fsm_wrtid),
      .mem_wradr_i(fsm_wradr),

      .mem_rdreq_i(fsm_rdreq),
      .mem_rdack_o(fsm_rdack),
      .mem_rderr_o(fsm_rderr),
      .mem_rdtid_i(fsm_rdtid),
      .mem_rdadr_i(fsm_rdadr),

      .cfg_req_i(cfg_req),  // Configuration port
      .cfg_run_i(cfg_run),  // Configuration port
      .cfg_rdy_o(cfg_rdy),
      .cfg_cmd_i(cfg_cmd),
      .cfg_ref_i(cfg_ref),
      .cfg_ba_i (cfg_ba),
      .cfg_adr_i(cfg_adr),

      .ddl_req_o(ddl_req),  // Controller <-> DFI
      .ddl_rdy_i(ddl_rdy),
      .ddl_cmd_o(ddl_cmd),
      .ddl_tid_o(ddl_tid),
      .ddl_ba_o (ddl_ba),
      .ddl_adr_o(ddl_adr)
  );


endmodule  // ddr3_fsm_tb
