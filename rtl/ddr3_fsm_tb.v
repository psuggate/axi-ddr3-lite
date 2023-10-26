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

  localparam ADDRS = DDR_COL_BITS + DDR_ROW_BITS + 3 - 1;
  localparam ASB = ADDRS - 1;

  localparam REQID = 4;
  localparam ISB = REQID - 1;


  // -- Simulation Data -- //

  initial begin
    $dumpfile("ddr3_fsm_tb.vcd");
    $dumpvars;

    #8000 $finish;  // todo ...
  end


  // -- Globals -- //

  reg osc = 1'b1;
  reg ddr = 1'b1;
  reg rst = 1'b0;

  always #5.0 osc <= ~osc;
  always #2.5 ddr <= ~ddr;

  initial begin
    rst <= 1'b1;
    #20 rst <= 1'b0;
  end


  wire locked, clock, reset;
  wire clk_ddr, clk_ddr_dqs, clk_ref;

  assign #10 locked = 1'b1;
  assign clk_ddr = ddr;
  assign clock = osc;
  assign reset = rst | ~locked;


  reg fsm_wrreq, fsm_wrlst;
  wire fsm_wrack, fsm_wrerr;
  reg fsm_rdreq, fsm_rdlst;
  wire fsm_rdack, fsm_rderr;
  reg [ISB:0] fsm_wrtid, fsm_rdtid;
  reg [ASB:0] fsm_wradr, fsm_rdadr;

  // AXI <-> {FSM, DDL} signals
  wire wr_valid, wr_ready, wr_last;
  wire rd_valid, rd_ready, rd_last;
  wire [SSB:0] wr_mask;
  wire [MSB:0] wr_data, rd_data;

  // DFI <-> PHY
  wire dfi_rst_n, dfi_ras_n, dfi_cas_n, dfi_we_n;
  wire dfi_odt, dfi_wren, dfi_rden, dfi_valid;
  wire [  2:0] dfi_bank;
  wire [RSB:0] dfi_addr;
  wire [SSB:0] dfi_mask;
  wire [MSB:0] dfi_wdata, dfi_rdata;


  // -- DDR3 Memory Controller FSM -- //

reg ddl_cke;
  wire ddl_rdy, ddl_req, ddl_ref;
  wire [2:0] ddl_cmd, ddl_ba;
  wire [ISB:0] ddl_tid;
  wire [RSB:0] ddl_adr;

  wire cfg_req, cfg_rdy, cfg_ref;
  wire [2:0] cfg_cmd, cfg_ba;
  wire [RSB:0] cfg_adr;

  reg  [  6:0] counter;
  reg  cfg_run;

  assign cfg_req = 1'b0;
  assign cfg_ref = 1'b0;

  always @(posedge clock) begin
    if (reset) begin
      cfg_run <= 1'b0;
      ddl_cke <= 1'b0;
      counter <= 7'd10;
      fsm_wrreq <= 1'b0;
      fsm_wrlst <= 1'b0;
      fsm_rdreq <= 1'b0;
      fsm_rdlst <= 1'b0;
    end else if (!cfg_run && counter > 0) begin
      ddl_cke <= 1'b1;
      counter <= counter - 1;
      cfg_run <= counter == 7'd001;
    end
  end


  initial begin
    while (!reset) begin
      @(posedge clock);
    end
    @(posedge clock);
    while (reset) begin
      @(posedge clock);
    end

    @(posedge clock);
    while (!cfg_run) begin
      @(posedge clock);
    end

    @(posedge clock);
    mem_store(0, 'bx);

    @(posedge clock);
    @(posedge clock);
  end


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

      .ddr_cke_i(ddl_cke),
      .ddr_cs_ni(~ddl_cke),

      .ctl_req_i(ddl_req),
      .ctl_rdy_o(ddl_rdy),
      .ctl_cmd_i(ddl_cmd),
      .ctl_ba_i (ddl_ba),
      .ctl_adr_i(ddl_adr),

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

  ddr3_fsm #(
      .DDR_FREQ_MHZ(DDR_FREQ_MHZ),
      .DDR_ROW_BITS(DDR_ROW_BITS),
      .DDR_COL_BITS(DDR_COL_BITS),
      .REQID(REQID),
      .ADDRS(ADDRS)
  ) ddr3_fsm_inst (
      .clock(clock),
      .reset(reset),

      .mem_wrreq_i(fsm_wrreq),  // Bus -> Controller requests
      .mem_wrlst_i(fsm_wrlst),
      .mem_wrack_o(fsm_wrack),
      .mem_wrerr_o(fsm_wrerr),
      .mem_wrtid_i(fsm_wrtid),
      .mem_wradr_i(fsm_wradr),

      .mem_rdreq_i(fsm_rdreq),
      .mem_rdlst_i(fsm_rdlst),
      .mem_rdack_o(fsm_rdack),
      .mem_rderr_o(fsm_rderr),
      .mem_rdtid_i(fsm_rdtid),
      .mem_rdadr_i(fsm_rdadr),

      .cfg_run_i(cfg_run),  // Configuration port
      .cfg_req_i(cfg_req),
      .cfg_rdy_o(cfg_rdy),
      .cfg_cmd_i(cfg_cmd),
      .cfg_ref_i(cfg_ref),
      .cfg_ba_i (cfg_ba),
      .cfg_adr_i(cfg_adr),

      .ddl_req_o(ddl_req),  // Controller <-> DFI
      .ddl_rdy_i(ddl_rdy),
                   .ddl_ref_i(cfg_ref),
      .ddl_cmd_o(ddl_cmd),
      .ddl_tid_o(ddl_tid),
      .ddl_ba_o (ddl_ba),
      .ddl_adr_o(ddl_adr)
  );


  // -- Perform write transfer (128-bit) -- //

  task mem_store;
    input [ASB:0] addr;
    input [127:0] data;
    begin
      integer count;

      fsm_wrreq <= 1'b1;
      fsm_wrlst <= 1'b1;
      fsm_wrtid <= 2'b00;
      fsm_wradr <= addr;

      count <= 0;

      @(posedge clock);

      while (!fsm_wrack) begin
        @(posedge clock);
      end

      fsm_wrreq <= 1'b0;
      @(posedge clock);

      count <= 1;

      @(posedge clock);
    end
  endtask  // axi_fetch


  // -- Perform read transfer (128-bit) -- //

  /*
  task axi_fetch;
    input [ASB:0] addr;
    output [127:0] data;
    begin
      arvalid <= 1'b1;
      arlen <= 128 / WIDTH - 1;
      arid <= arid + 1;
      arburst <= 2'b01;  // INCR
      araddr <= addr;
      rready <= 1'b0;

      @(posedge clock);

      while (!arready) begin
        @(posedge clock);
      end
      arvalid <= 1'b0;
      rready  <= 1'b1;

      @(posedge clock);

      while (!rvalid || !rlast) begin
        if (rvalid) begin
          data <= {rdata, data[127:WIDTH]};
        end

        @(posedge clock);
      end

      rready <= 1'b0;
      data   <= {rdata, data[127:WIDTH]};
      @(posedge clock);
    end
  endtask  // axi_fetch
*/

endmodule  // ddr3_fsm_tb
