`timescale 1ns / 100ps
module ddr3_ddl_tb;

  // -- Simulation Settings -- //

  parameter ADDRS = 32;
  localparam ASB = ADDRS - 1;

  parameter WIDTH = 32;
  localparam MSB = WIDTH - 1;

  parameter MASKS = WIDTH / 8;
  localparam SSB = MASKS - 1;

  parameter DDR3_ROW_BITS = 15;
  localparam RSB = DDR3_ROW_BITS - 1;
  parameter DDR3_COL_BITS = 10;
  localparam CSB = DDR3_COL_BITS - 1;

  localparam DSB = WIDTH / 2 - 1;
  localparam QSB = MASKS / 2 - 1;


  // -- Constants -- //

  localparam DDR3_ACTV = 3'b011;
  localparam DDR3_READ = 3'b010;


  // -- Simulation Data -- //

  initial begin
    $dumpfile("ddr3_ddl_tb.vcd");
    $dumpvars();

    #800 $finish;  // todo ...
  end


  // -- Globals -- //

  reg osc = 1'b1;
  reg ddr = 1'b1;
  reg rst = 1'b0;

  always #5.0 osc <= ~osc;
  always #2.5 ddr <= ~ddr;

  initial begin
    rst <= 1'b1;
    #60 rst <= 1'b0;
  end


  wire locked, clock, reset;
  wire clk_ddr, clk_ddr_dqs, clk_ref;


  assign clock = osc;
  assign reset = rst | ~locked;

  assign #50 locked = 1'b1;


  wire ddr_ck_p, ddr_ck_n, ddr_odt;
  wire ddr_cke, ddr_rst_n, ddr_cs_n, ddr_ras_n, ddr_cas_n, ddr_we_n;
  wire [  2:0] ddr_ba;
  wire [RSB:0] ddr_a;
  wire [QSB:0] ddr_dqs_p, ddr_dqs_n, ddr_dm;
  wire [DSB:0] ddr_dq;

  wire dfi_cke, dfi_rst_n, dfi_cs_n, dfi_ras_n, dfi_cas_n, dfi_we_n;
  wire dfi_rden, dfi_wren, dfi_valid;
  wire [  2:0] dfi_bank;
  wire [RSB:0] dfi_addr;
  wire [SSB:0] dfi_mask;
  wire [MSB:0] dfi_rdata, dfi_wdata;

  reg en, req, pre;
  reg [  2:0] cmd;
  reg [  2:0] ba;
  reg [RSB:0] ad;


  // -- Stimulus -- //

  reg [MSB:0] data;

  initial begin : STIMULATE
    ddr_actv(0, 0);
    ddr_read(0, 0);
  end  // STIMULATE


  always @(posedge clock) begin
    if (reset) begin
      req <= 1'b0;
      en  <= 1'b0;
      pre <= 1'b0;
    end else begin
      en <= 1'b1;  // todo: wait > 5 cycles (for CK to stabilise) ??
    end
  end


  // -- Module Under Test -- //

  wire rfc, rdy;

  ddr3_ddl #(
      .DDR_FREQ_MHZ  (100),
      .DDR_ROW_BITS  (DDR3_ROW_BITS),
      .DDR_COL_BITS  (DDR3_COL_BITS),
      .DFI_DATA_WIDTH(WIDTH)
  ) ddr3_ddl_inst (
      .clock(clock),
      .reset(reset),

      .ctl_req_i(req),
      .ctl_rdy_o(rdy),
      .ctl_ref_o(rfc),
      .ctl_cmd_i(cmd),
      .ctl_ba_i (ba),
      .ctl_adr_i(ad),

      .dfi_rst_no (dfi_rst_n),
      .dfi_cke_o  (dfi_cke),
      .dfi_cs_no  (dfi_cs_n),
      .dfi_ras_no (dfi_ras_n),
      .dfi_cas_no (dfi_cas_n),
      .dfi_we_no  (dfi_we_n),
      .dfi_odt_o  (),
      .dfi_bank_o (dfi_bank),
      .dfi_addr_o (dfi_addr),
      .dfi_wren_o (dfi_wren),
      .dfi_mask_o (dfi_mask),
      .dfi_data_o (dfi_wdata),
      .dfi_rden_o (dfi_rden),
      .dfi_valid_i(dfi_valid),
      .dfi_data_i (dfi_rdata)
  );


  // -- PHY for the DDR3 -- //

  generic_ddr3_phy #(
      .DDR3_WIDTH(16),
      .DDR3_MASKS(2),
      .ADDR_BITS (13)
  ) generic_ddr3_phy_inst (
      .clock(clock),
      .reset(reset),

      .dfi_cke_i (dfi_cke),
      .dfi_data_o(dfi_rdata),

      .ddr3_ck_po(ddr_ck_p),
      .ddr3_ck_no(ddr_ck_n),
      .ddr3_cke_o(ddr_cke),
      // todo ...
      .ddr3_dq_io(ddr_dq)
  );


  // -- Micron's DDR3 Simulation Module -- //

  ddr3 ddr3_inst (
      .ck  (ddr_ck_p),
      .ck_n(ddr_ck_n),
      .cke (ddr_cke),
      // todo ...
      .dq  (ddr_dq)
  );


  // -- DDR3 Row ACTIVATE Command -- //

  task ddr_actv;
    input [2:0] bank;
    input [RSB:0] row;
    begin
      cmd <= DDR3_ACTV;
      req <= 1'b1;
      pre <= 1'b0;
      ba  <= bank;
      ad  <= row;

      @(posedge clock);
      while (!rdy) begin
        @(posedge clock);
      end

      req <= 1'b0;
      @(posedge clock);
    end
  endtask  // ddr_actv


  // -- DDR3 READ Command -- //

  task ddr_read;
    input [2:0] bank;
    input [CSB:0] col;
    begin
      cmd <= DDR3_READ;
      req <= 1'b1;
      pre <= 1'b0;
      ba  <= bank;
      ad  <= {4'h0, pre, col};  // todo:

      @(posedge clock);
      while (!rdy) begin
        @(posedge clock);
      end

      req <= 1'b0;
      @(posedge clock);
    end
  endtask  // ddr_read


endmodule  // ddr3_ddl_tb
