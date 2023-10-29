`timescale 1ns / 100ps
// `define __gowin_for_the_win
module axi_ddr3_lite_tb;

  // -- Simulation Settings -- //

  localparam DDR_FREQ_MHZ = 100;
  `include "ddr3_settings.vh"

  localparam DDR_ROW_BITS = 13;
  localparam RSB = DDR_ROW_BITS - 1;

  parameter DDR_COL_BITS = 10;
  localparam CSB = DDR_COL_BITS - 1;

  // Trims an additional clock-cycle of latency, if '1'
  parameter LOW_LATENCY = 1'b1;  // 0 or 1

  // -- Data-path and address settings -- //

  localparam WIDTH = 32;
  localparam MSB = WIDTH - 1;
  localparam HSB = WIDTH / 2 - 1;

  localparam MASKS = WIDTH / 8;
  localparam SSB = MASKS - 1;
  localparam QSB = MASKS / 2 - 1;

  // note: (AXI4) byte address, not burst-aligned address
  localparam ADDRS = DDR_COL_BITS + DDR_ROW_BITS + 4;
  localparam ASB = ADDRS - 1;

  localparam REQID = 4;
  localparam ISB = REQID - 1;


  // -- Simulation Data -- //

  initial begin
    $dumpfile("axi_ddr3_lite_tb.vcd");
    $dumpvars(0, axi_ddr3_lite_tb);

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

`ifdef __gowin_for_the_win
  // todo: work out actual values ...
  gowin_rpll #(
      .FCLKIN("27"),
      .IDIV_SEL(8),
      .FBDIV_SEL(17),
      .ODIV_SEL(3)
  ) RPLL_inst (
      .clkin (osc),
      .lock  (locked),
      .clkout(clk_ddr),
      .clkref(clk_ref)
  );

`else

  assign #50 locked = 1'b1;
  assign clk_ddr = ddr;

`endif

  assign clock = osc;
  assign reset = rst | ~locked;


  // -- DDR3 and Controller Signals -- //

  // DFI <-> PHY
  wire dfi_rst_n, dfi_cke, dfi_cs_n, dfi_ras_n, dfi_cas_n, dfi_we_n;
  wire dfi_odt, dfi_wstb, dfi_wren, dfi_rden, dfi_valid;
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
  wire [QSB:0] ddr_dm, ddr_dqs_p, ddr_dqs_n;
  wire [HSB:0] ddr_dq;

  // AXI4 Signals to/from the Memory Controller
  reg awvalid, wvalid, wlast, bready, arvalid, rready;
  reg abvalid, dbready;
  reg accept, error, rd_valid, rd_last, wr_ready;
  reg [ISB:0] awid, arid, byid, respi;
  reg [7:0] awlen, arlen, bylen;
  reg [1:0] awburst, arburst, byburst;
  reg [ASB:0] awaddr, araddr, byaddr;
  // reg [MSB:0] rd_data;
  reg [SSB:0] wstrb;
  wire awready, wready, bvalid, arready, rvalid, rlast, fetch, store, rd_ready, wr_valid, wr_last;
  wire abready, dbvalid, dblast;
  wire [ISB:0] bid, rid, dbid, reqid;
  wire [1:0] bresp, rresp, dbresp;
  wire [ASB:0] maddr;
  wire [SSB:0] wr_mask;
  wire [MSB:0] rdata, bdata, rd_data, wr_data;


  // -- Initialisation -- //

  reg [127:0] data;
  reg [MSB:0] wdata;

  initial begin : Stimulus
    @(posedge clock);

    while (reset) begin
      @(posedge clock);

      awvalid <= 1'b0;
      wvalid <= 1'b0;
      wlast <= 1'b0;
      awid <= 0;
      awaddr <= 0;
      wstrb <= 0;
      bready <= 1'b1;

      rready <= 1'b0;
      arvalid <= 1'b0;
      arid <= 0;
      araddr <= 0;
    end

    while (!awready || !arready) begin
      @(posedge clock);
    end

    @(posedge clock);
    @(posedge clock);
    data <= {$urandom, $urandom, $urandom, $urandom};

    @(posedge clock);

    axi_store(0, data);
    $display("TB:%10t: WRITE = %x", $time, data);

    @(posedge clock);
    while (!bvalid || !bready) begin
      @(posedge clock);
    end
    @(posedge clock);

    axi_fetch(0, data);
    $display("TB:%10t: READ = %x", $time, data);

    #40 @(posedge clock);
    $finish;
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


  // -- DDR3 PHY -- //

`ifdef __gowin_for_the_win

  // GoWin Global System Reset signal tree.
  GSR GSR ();

  // generic_ddr3_dfi_phy
  gowin_ddr3_dfi_phy #(
      .DDR3_WIDTH(16),  // (default)
      .ADDR_BITS(15),  // default: 14
      .SOURCE_CLOCK(2'b01),  // (default)
      .CAPTURE_DELAY(3'h0)  // (default)
  ) u_phy (
      .clock  (clock),
      .reset  (reset),
      .clk_ddr(clk_ddr),

      .cfg_valid_i(1'b0),
      .cfg_data_i ({16'h0000, 4'h4, 4'h4, 8'h00}),

      .dfi_cke_i(dfi_cke),
      .dfi_reset_n_i(dfi_rst_n),
      .dfi_cs_n_i(dfi_cs_n),
      .dfi_ras_n_i(dfi_ras_n),
      .dfi_cas_n_i(dfi_cas_n),
      .dfi_we_n_i(dfi_we_n),
      .dfi_odt_i(dfi_odt),
      .dfi_bank_i(dfi_bank),
      .dfi_addr_i(dfi_addr),

      .dfi_wren_i(dfi_wren),
      .dfi_mask_i(dfi_mask),
      .dfi_data_i(dfi_wdata),

      .dfi_rden_i (dfi_rden),
      .dfi_valid_o(dfi_valid),
      .dfi_data_o (dfi_rdata),

      .ddr3_ck_p_o(ddr_ck_p_w),
      .ddr3_ck_n_o(ddr_ck_n_w),
      .ddr3_cke_o(ddr_cke_w),
      .ddr3_reset_n_o(ddr_reset_n_w),
      .ddr3_cs_n_o(ddr_cs_n_w),
      .ddr3_ras_n_o(ddr_ras_n_w),
      .ddr3_cas_n_o(ddr_cas_n_w),
      .ddr3_we_n_o(ddr_we_n_w),
      .ddr3_odt_o(ddr_odt_w),
      .ddr3_ba_o(ddr_ba_w),
      .ddr3_a_o(ddr_addr_w),
      .ddr3_dm_o(ddr_dm_w),
      .ddr3_dqs_p_io(ddr_dqs_p_w),
      .ddr3_dqs_n_io(ddr_dqs_n_w),
      .ddr3_dq_io(ddr_dq_w)
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
      .DDR_DQ_WIDTH (WIDTH / 2),
      .LOW_LATENCY  (LOW_LATENCY ),
      .AXI_ID_WIDTH (REQID),
      .MEM_ID_WIDTH (REQID),
      .BYPASS_ENABLE(0)
  ) ddr_core_inst (
      .clock(clock),  // system clock
      .reset(reset),  // synchronous reset

      .axi_awvalid_i(awvalid),
      .axi_awready_o(awready),
      // .axi_awaddr_i(awaddr),
      .axi_awaddr_i(awaddr[ASB:4]),  // todo
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
      // .axi_araddr_i(araddr),
      .axi_araddr_i(araddr[ASB:4]),  // todo
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
      // .byp_araddr_i(byaddr),
      .byp_araddr_i(byaddr[ASB:4]),  // todo
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
      .dfi_data_i(dfi_rdata)
  );


  //
  //  Simulation Tasks
  ///

  // -- Perform write transfer (128-bit) -- //

  task axi_store;
    input [ASB:0] addr;
    input [127:0] data;
    begin
      integer count;

      awvalid <= 1'b1;
      awlen <= 128 / WIDTH - 1;
      awid <= arid + 1;
      awburst <= 2'b01;  // INCR
      awaddr <= addr;
      wvalid <= 1'b0;
      count <= 0;

      @(posedge clock);

      while (!awready) begin
        @(posedge clock);
      end

      awvalid <= 1'b0;
      wvalid  <= 1'b1;
      wlast   <= 1'b0;
      wstrb <= 4'hf;
      wdata   <= data[MSB:0];
      data    <= {{WIDTH{1'bx}}, data[127:WIDTH]};
      count   <= 1;

      while (!wready || count < 4) begin
        if (wready) begin
          count <= count + 1;
          wlast <= count > 2;
          wdata <= data[MSB:0];
          data  <= {{WIDTH{1'bx}}, data[127:WIDTH]};
        end

        @(posedge clock);
      end

      wvalid <= 1'b0;
      wlast  <= 1'b0;
    end
  endtask  // axi_fetch


  // -- Perform read transfer (128-bit) -- //

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


endmodule  // axi_ddr3_lite
