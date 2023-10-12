`timescale 1ns / 100ps
module ddr3_axi_ctrl_tb;

  // -- Simulation Settings -- //

  parameter ADDRS = 32;
  localparam ASB = ADDRS - 1;

  parameter WIDTH = 32;
  localparam MSB = WIDTH - 1;

  parameter MASKS = WIDTH / 8;
  localparam SSB = MASKS - 1;


  // -- Simulation Data -- //

  initial begin
    $dumpfile("ddr3_axi_ctrl_tb.vcd");
    $dumpvars(0, ddr3_axi_ctrl_tb);

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


  reg awvalid, wvalid, wlast, bready, arvalid, rready, accept, error, rd_valid, rd_last, wr_ready;
  reg [3:0] awid, arid, respi;
  reg [7:0] awlen, arlen;
  reg [1:0] awburst, arburst;
  reg [ASB:0] awaddr, araddr;
  // reg [MSB:0] rd_data;
  reg [SSB:0] wstrb;
  wire awready, wready, bvalid, arready, rvalid, rlast, fetch, store, rd_ready, wr_valid, wr_last;
  wire [3:0] bid, rid, reqid;
  wire [1:0] bresp, rresp;
  wire [ASB:0] maddr;
  wire [SSB:0] wr_mask;
  wire [MSB:0] rdata, rd_data, wr_data;


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


  // -- Fake SDRAM -- //

  localparam MBITS = 10;
  localparam MSIZE = 1 << MBITS;

  reg [7:0] fram0[0:MSIZE-1];
  reg [7:0] fram1[0:MSIZE-1];
  reg [7:0] fram2[0:MSIZE-1];
  reg [7:0] fram3[0:MSIZE-1];

  reg [MBITS-1:0] wr_addr, rd_addr;

  integer ii;

  initial begin : FAKE_MEMORIES_ARE_ALL_I_HAVE
    for (ii = 0; ii < MSIZE; ii = ii + 1) begin
      {fram3[ii], fram2[ii], fram1[ii], fram0[ii]} = $urandom;
    end
  end  // FAKE_MEMORIES_ARE_ALL_I_HAVE


  // Control
  always @(posedge clock) begin
    if (reset) begin
      error  <= 1'b0;
      accept <= 1'b0;
    end else begin
      if (accept && (fetch || store)) begin
        accept <= 1'b0;
      end else begin
        accept <= 1'b1;
      end
    end
  end

  // Store
  always @(posedge clock) begin
    if (reset) begin
      wr_ready <= 1'b0;
      wr_addr  <= {MBITS{1'bx}};
    end else begin
      if (store && accept) begin
        wr_ready <= 1'b1;
        wr_addr  <= maddr[MBITS+1:2];

        if (maddr[1:0] != 2'b00) begin
          $error("TB:%10t: Unaligned WRITE not supported, LSBs: %02b", $time, maddr[1:0]);
          $fatal;
        end
      end else if (wr_valid && wr_ready && wr_last) begin
        wr_ready <= 1'b0;
        $display("TB:%10t: WRITE has completed", $time);
      end

      if (wr_valid && wr_ready) begin
        if (wr_mask[0]) fram0[wr_addr] <= wr_data[7:0];
        if (wr_mask[1]) fram1[wr_addr] <= wr_data[15:8];
        if (wr_mask[2]) fram2[wr_addr] <= wr_data[23:16];
        if (wr_mask[3]) fram3[wr_addr] <= wr_data[31:24];
        wr_addr <= wr_addr + 1;
      end
    end
  end

  // Fetch
  reg  [1:0] rd_count;
  wire [1:0] rd_cnext = rd_count - 1;

  assign rd_data = {fram3[rd_addr], fram2[rd_addr], fram1[rd_addr], fram0[rd_addr]};

  always @(posedge clock) begin
    if (reset) begin
      rd_valid <= 1'b0;
      rd_addr  <= {MBITS{1'bx}};
      rd_last  <= 1'b0;
    end else begin
      if (fetch && accept) begin
        rd_valid <= 1'b1;
        rd_addr <= maddr[MBITS+1:2];
        rd_count <= 2'd3;
        respi <= reqid;

        if (maddr[1:0] != 2'b00) begin
          $error("TB:%10t: Unaligned WRITE not supported, LSBs: %02b", $time, maddr[1:0]);
          $fatal;
        end
      end else if (rd_valid && rd_ready && rd_last) begin
        rd_valid <= 1'b0;
        rd_last <= 1'b0;
        respi <= 4'hx;
      end else if (rd_valid && rd_ready) begin
        rd_addr <= rd_addr + 1;

        if (rd_cnext > 0) begin
          rd_count <= rd_cnext;
        end else begin
          rd_last <= 1'b1;
        end
      end
    end
  end


  // -- Module Under Test -- //

  ddr3_axi_ctrl #(
      .WIDTH(WIDTH),
      .MASKS(MASKS),
      .ADDRS(ADDRS)
  ) ddr3_axi_ctrl_inst (
      .clock(clock),
      .reset(reset),

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

      .mem_store_o (store),
      .mem_fetch_o (fetch),
      .mem_accept_i(accept),
      .mem_error_i (error),
      .mem_req_id_o(reqid),
      .mem_addr_o  (maddr),

      .mem_valid_o (wr_valid),
      .mem_ready_i (wr_ready),
      .mem_last_o  (wr_last),
      .mem_wrmask_o(wr_mask),
      .mem_wrdata_o(wr_data),

      .mem_valid_i(rd_valid),
      .mem_ready_o(rd_ready),
      .mem_last_i(rd_last),
      .mem_resp_id_i(respi),
      .mem_rddata_i(rd_data)
  );


  //-----------------------------------------------------------------
  // axi_store: Perform write transfer (128-bit)
  //-----------------------------------------------------------------
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


  //-----------------------------------------------------------------
  // axi_fetch: Perform read transfer (128-bit)
  //-----------------------------------------------------------------
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


// -- Another Module Yet Not Under Test -- //

`ifdef __amynut
  assign store = |wr_mask;

  ddr3_axi_pmem ddr3_axi_pmem_inst (
      .clock(clock),
      .reset(reset),

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

      .ram_accept_i(accept),
      .ram_error_i (error),
      .ram_req_id_o(reqid),
      .ram_addr_o  (maddr),

      .ram_wr_o(wr_mask),
      .ram_write_data_o(wr_data),

      .ram_rd_o (fetch),
      .ram_ack_i(rd_valid),
      .ram_resp_id_i(respi),
      .ram_read_data_i(rd_data)
  );
`endif


endmodule  // ddr3_axi_ctrl_tb
