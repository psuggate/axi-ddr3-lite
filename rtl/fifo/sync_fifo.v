`timescale 1ns / 100ps
module sync_fifo (
    clock,
    reset,

    level_o,

    valid_i,
    ready_o,
    data_i,

    valid_o,
    ready_i,
    data_o
);

  // Skid-buffer for the output data, so that registered-output SRAM's can be
  // used; e.g., Xilinx Block SRAMs, or GoWin BSRAMs.
  // Notes:
  //  - OUTREG = 0 for a FIFO that supports LUT-SRAMs with asynchronous reads
  //  - OUTREG = 1 for the smallest block-SRAM FIFO
  //  - OUTREG = 2 for a block-SRAM FIFO with First-Word Fall-Through (FWFT)
  parameter OUTREG = 1;  // 0, 1, or 2

  parameter WIDTH = 8;
  localparam MSB = WIDTH - 1;

  parameter ABITS = 4;
  localparam DEPTH = 1 << ABITS;
  localparam ASB = ABITS - 1;
  localparam ADDRS = ABITS + 1;
  localparam AZERO = {ABITS{1'b0}};


  input clock;
  input reset;

  output [ASB:0] level_o;

  input valid_i;
  output ready_o;
  input [MSB:0] data_i;

  output valid_o;
  input ready_i;
  output [MSB:0] data_o;


  reg [MSB:0] sram[0:DEPTH-1];

  // Write-port signals
  reg wready;
  reg [ABITS:0] waddr;
  wire [ABITS:0] waddr_next;

  // Read-port signals
  reg rvalid;
  reg [ABITS:0] raddr;
  wire [ABITS:0] raddr_next;

  // Transition signals
  reg [ASB:0] level_q;
  wire fetch_w, store_w, match_w, wfull_w, empty_w;
  wire [ABITS:0] level_w, waddr_w, raddr_w;


  assign level_o = level_q;
  assign ready_o = wready;


  // -- FIFO Status Signals -- //

  wire wrfull_next = waddr_next[ASB:0] == raddr[ASB:0] && store_w && !fetch_w;
  wire wrfull_curr = match_w && waddr[ABITS] != raddr[ABITS] && fetch_w == store_w;

  wire rempty_next = raddr_next[ASB:0] == waddr[ASB:0] && fetch_w && !store_w;
  wire rempty_curr = match_w && waddr[ABITS] == raddr[ABITS] && fetch_w == store_w;

  assign #3 match_w = waddr[ASB:0] == raddr[ASB:0];
  assign #3 wfull_w = wrfull_curr || wrfull_next;
  assign #3 empty_w = rempty_curr || rempty_next;

  assign level_w = waddr_w[ASB:0] - raddr_w[ASB:0];
  assign waddr_w = store_w ? waddr_next : waddr;
  assign raddr_w = fetch_w ? raddr_next : raddr;


  // -- Write Port -- //

  assign waddr_next = waddr + 1;

  always @(posedge clock) begin
    if (reset) begin
      waddr  <= {ADDRS{1'b0}};
      wready <= 1'b0;
    end else begin
      wready <= ~wfull_w;

      if (store_w) begin
        sram[waddr] <= data_i;
        waddr <= waddr_next;
      end
    end
  end


  // -- Read Port -- //

  assign raddr_next = raddr + 1;

  always @(posedge clock) begin
    if (reset) begin
      raddr  <= {ADDRS{1'b0}};
      rvalid <= 1'b0;
    end else begin
      if (empty_w && fetch_w) begin
        rvalid <= 1'b0;
        // end else if (!empty_w && !fetch_w) begin
      end else if (!empty_w) begin
        rvalid <= 1'b1;
      end

      if (fetch_w) begin
        raddr <= raddr_next;
      end
    end
  end


  // -- FIFO Status -- //

  always @(posedge clock) begin
    if (reset) begin
      level_q <= AZERO;
    end else begin
      level_q <= level_w[ASB:0];
    end
  end


  // -- Output Register -- //

  generate
    if (OUTREG == 0) begin : g_async

      // Suitable for Xilinx Distributed SRAM's, and similar, with fast, async
      // reads.
      assign #3 store_w = wready && valid_i;
      assign #3 fetch_w = rvalid && ready_i;

      assign valid_o = rvalid;
      assign data_o = sram[raddr[ASB:0]];

    end // g_async
  else if (OUTREG > 0) begin : g_outregs

      // Add an extra stage of registers, so that block SRAMs can be used.
      reg xvalid, empty_q;
      wire xready, svalid, sready;
      reg [MSB:0] xdata, sdata;

      assign #3 store_w = wready && valid_i &&
                          (OUTREG < 2 || xvalid || !empty_q || svalid && !sready);
      assign #3 fetch_w = rvalid && (svalid && sready || !svalid);

      always @(posedge clock) begin
        if (reset) begin
          xvalid <= 1'b0;
        end else begin
          if (fetch_w) begin
            xvalid <= 1'b1;
            xdata  <= sram[raddr[ASB:0]];
          end else if (xvalid && xready) begin
            xvalid <= 1'b0;
          end
        end
      end


      // -- First-Word Fall-Through -- //

      assign xready = sready;
      assign svalid = xvalid || !xvalid && valid_i && OUTREG > 1;
      assign sdata  = !xvalid && valid_i && sready && OUTREG > 1 ? data_i : xdata;

      always @(posedge clock) begin
        empty_q <= empty_w;
      end

      axis_skid #(
          .WIDTH (WIDTH),
          .BYPASS(OUTREG > 1 ? 0 : 1)
      ) axis_skid_inst (
          .clock(clock),
          .reset(reset),

          .s_tvalid(svalid),
          .s_tready(sready),
          .s_tlast (1'b0),
          .s_tdata (sdata),

          .m_tvalid(valid_o),
          .m_tready(ready_i),
          .m_tlast (),
          .m_tdata (data_o)
      );

    end  // g_outregs
  endgenerate


endmodule  // sync_fifo
