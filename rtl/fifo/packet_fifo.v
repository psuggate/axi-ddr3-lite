`timescale 1ns / 100ps
module packet_fifo (
    clock,
    reset,

    valid_i,
    ready_o,
    last_i,
    drop_i,
    data_i,

    valid_o,
    ready_i,
    last_o,
    data_o
);

  // Skid-buffer for the output data, so that registered-output SRAM's can be
  // used; e.g., Xilinx Block SRAMs, or GoWin BSRAMs.
  parameter OUTREG = 1;  // 0, 1, or 2

  parameter WIDTH = 8;
  localparam MSB = WIDTH - 1;

  parameter ABITS = 4;
  localparam DEPTH = 1 << ABITS;
  localparam ASB = ABITS - 1;
  localparam ADDRS = ABITS + 1;


  input clock;
  input reset;

  input valid_i;
  output ready_o;
  input last_i;
  input drop_i;
  input [MSB:0] data_i;

  output valid_o;
  input ready_i;
  output last_o;
  output [MSB:0] data_o;


  reg [WIDTH:0] sram[0:DEPTH-1];

  // Write-port signals
  reg wready, wlast;
  reg [ABITS:0] waddr;
  wire [ABITS:0] waddr_next;

  // Read-port signals
  reg rvalid;
  reg [ABITS:0] raddr;
  wire [ABITS:0] raddr_next;

  // Packet address signals
  reg [ABITS:0] paddr;
  wire [ABITS:0] paddr_next;

  // Transition signals
  wire fetch, store, match, wfull, empty, reject, accept;


  assign ready_o = wready;
  assign last_o  = wlast;


  // -- FIFO Status Signals -- //

  wire wrfull_next = waddr_next[ASB:0] == raddr[ASB:0] && store && !fetch;
  wire wrfull_curr = match && waddr[ABITS] != raddr[ABITS] && fetch == store;

  wire rempty_next = raddr_next[ASB:0] == paddr[ASB:0] && fetch && !accept;
  wire rempty_curr = paddr == raddr && fetch == accept;


  assign accept = valid_i && last_i && !drop_i && ready_o;
  assign reject = drop_i;  // ??

  assign store = wready && valid_i;
  assign match = waddr[ASB:0] == raddr[ASB:0];
  assign wfull = wrfull_curr || wrfull_next;
  assign empty = rempty_curr || rempty_next;


  // -- Write Port -- //

  assign waddr_next = waddr + 1;

  always @(posedge clock) begin
    if (reset) begin
      waddr  <= {ADDRS{1'b0}};
      wready <= 1'b0;
    end else begin
      wready <= ~wfull;

      if (reject) begin
        waddr <= paddr;
      end else if (valid_i && wready) begin
        sram[waddr] <= {last_i, data_i};
        waddr <= waddr_next;
      end
    end
  end


  // -- Frame Pointer -- //

  assign paddr_next = paddr + 1;

  always @(posedge clock) begin
    if (reset) begin
      paddr <= {ADDRS{1'b0}};
    end else begin
      if (accept) begin
        paddr <= waddr_next;
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
      if (empty && fetch) begin
        rvalid <= 1'b0;
      end else if (!empty && !fetch) begin
        rvalid <= 1'b1;
      end

      if (fetch) begin
        raddr <= raddr_next;
      end
    end
  end


  // -- Output Register -- //

  generate
    if (OUTREG == 0) begin : g_async

      wire [WIDTH:0] data_w = sram[raddr[ASB:0]];

      // Suitable for Xilinx Distributed SRAM's, and similar, with fast, async
      // reads.
      assign fetch   = rvalid && ready_i;

      assign valid_o = rvalid;
      assign last_o  = data_w[WIDTH];
      assign data_o  = data_w[MSB:0];

    end // g_async
  else if (OUTREG > 0) begin : g_outregs

      // Add an extra stage of registers, so that block SRAMs can be used.
      reg xvalid, xlast;
      wire xready;
      reg [MSB:0] xdata;

      assign fetch = rvalid && (xvalid && xready || !xvalid);

      always @(posedge clock) begin
        if (reset) begin
          xvalid <= 1'b0;
        end else begin
          if (fetch) begin
            raddr <= raddr_next;
            xvalid <= 1'b1;
            {xlast, xdata} <= sram[raddr[ASB:0]];
          end else if (xvalid && xready) begin
            xvalid <= 1'b0;
          end
        end
      end

      axis_skid #(
          .WIDTH (WIDTH),
          .BYPASS(OUTREG > 1 ? 0 : 1)
      ) axis_skid_inst (
          .clock(clock),
          .reset(reset),

          .s_tvalid(xvalid),
          .s_tready(xready),
          .s_tlast (xlast),
          .s_tdata (xdata),

          .m_tvalid(valid_o),
          .m_tready(ready_i),
          .m_tlast (last_o),
          .m_tdata (data_o)
      );

    end  // g_outregs
  endgenerate


endmodule  // packet_fifo
