`timescale 1ns / 100ps
module gw2a_ddr_iob_tb;

reg clk_x1 = 1'b1;
reg clk_x2 = 1'b1;
reg reset  = 1'bx;

always #2.5 clk_x2 <= ~clk_x2;
always #5.0 clk_x1 <= ~clk_x1;

  initial begin
    #20 reset <= 1'b1;
    #40 reset <= 1'b0;
  end


reg oe_nq = 1'b1;
reg rnd_q;
reg [1:0] dat_q;
wire iob_w;
wire [1:0] dat_w;


  // -- Simulation Data -- //

  initial begin
    $dumpfile("gw2a_ddr_iob_tb.vcd");
    $dumpvars;

    #800 $finish;
  end


// -- Stimulus -- //

reg [3:0] count;
reg oe_pq, oe_qn;
wire [3:0] cnext = count + 1;
wire oe_nw = count > 1 && count < 6;
wire oe_nx = count > 7 && count < 12;

assign iob_w = oe_qn ? 1'bz : rnd_q;

always @(posedge clk_x1) begin
  if (reset) begin
    oe_nq <= 1'b1;
    oe_pq <= 1'b0;
    oe_qn <= 1'b1;
    count <= 0;
    dat_q <= 'bz;
  end else begin
    count <= cnext;
    oe_nq <= ~oe_nw;
    oe_pq <= oe_nx;
    oe_qn <= ~oe_pq;
    dat_q <= oe_nw ? $urandom : 'bz;
  end
end

always @(posedge clk_x2) begin
  rnd_q <= $urandom;
end


// -- Simulated Module Under Test -- //

gw2a_ddr_iob
#( .SHIFT(2'b00)
) u_gw2a_ddr_iob
(  .PCLK(clk_x1),
   .FCLK(clk_x2),
   .RESET(reset),
   .OEN(oe_nq),
   .D0(dat_q[0]),
   .D1(dat_q[1]),
   .Q0(dat_w[0]),
   .Q1(dat_w[1]),
   .IO(iob_w)
 );


endmodule // gw2a_ddr_iob_tb
