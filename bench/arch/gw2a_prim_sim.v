`timescale 1ns / 100ps
module GSR;

  reg GSRO = 1'b0;

  initial begin : BOOT
    #5 GSRO <= 1'b1;
  end

endmodule  // GSR

module IDDR(Q0, Q1, D, CLK);

input D;
input CLK;
output Q0;
output Q1;

parameter Q0_INIT = 1'b0;
parameter Q1_INIT = 1'b0;

wire gsrt = GSR.GSRO;

reg Q0_oreg, Q1_oreg,Q0_reg, Q1_reg;

initial begin
  Q0_reg = Q0_INIT;
  Q1_reg = Q1_INIT;
  Q0_oreg = Q0_INIT;
  Q1_oreg = Q1_INIT;
end

assign Q0 = Q0_reg;
assign Q1 = Q1_reg;

always @(gsrt) begin
  if(!gsrt) begin
    assign Q0_reg = Q0_INIT;
    assign Q1_reg = Q1_INIT;
    assign Q0_oreg = Q0_INIT;
    assign Q1_oreg = Q1_INIT;
  end
  else begin
    deassign Q0_reg;
    deassign Q1_reg;
    deassign Q0_oreg;
    deassign Q1_oreg;
  end
end

always @(posedge CLK) begin
  Q0_oreg <= D;
  Q0_reg <= Q0_oreg;
  Q1_reg <= Q1_oreg;

end

always @(negedge CLK) begin
  Q1_oreg <= D;
end

endmodule //IDDR (ddr input)

module ODDR (
    Q0,
    Q1,
    D0,
    D1,
    TX,
    CLK
);

  parameter TXCLK_POL = 1'b0;  //1'b0:Rising edge output; 1'b1:Falling edge output
  parameter INIT = 1'b0;

  input D0;
  input D1;
  input TX;
  input CLK;
  output Q0;
  output Q1;

  reg Dd0_0, Dd0_1, Dd0_2;
  reg Dd1_0, Dd1_1, Dd1_2;
  reg Ttx0, Ttx1, DT0, DT1;
  wire gsrt = GSR.GSRO;

  initial begin
    Dd0_0 = 1'b0;
    Dd0_1 = 1'b0;
    Dd0_2 = 1'b0;
    Dd1_0 = 1'b0;
    Dd1_1 = 1'b0;
    Dd1_2 = 1'b0;
    Ttx0  = 1'b0;
    Ttx1  = 1'b0;
    DT0   = 1'b0;
    DT1   = 1'b0;
  end

  always @(gsrt) begin
    if (!gsrt) begin
      assign Dd1_2 = INIT;
      assign Dd0_2 = INIT;
      assign Dd1_1 = INIT;
      assign Dd0_1 = INIT;
      assign Dd1_0 = INIT;
      assign Dd0_0 = INIT;
      assign Ttx0 = INIT;
      assign Ttx1 = INIT;
      assign DT0 = INIT;
      assign DT1 = INIT;
    end else begin
      deassign Dd1_2;
      deassign Dd0_2;
      deassign Dd1_1;
      deassign Dd0_1;
      deassign Dd1_0;
      deassign Dd0_0;
      deassign Ttx0;
      deassign Ttx1;
      deassign DT0;
      deassign DT1;
    end
  end

  always @(posedge CLK) begin
    Dd0_0 <= D0;
    Dd1_0 <= D1;
    Dd0_1 <= Dd0_0;
    Dd1_1 <= Dd1_0;
    Ttx0  <= TX;
    Ttx1  <= Ttx0;
  end

  always @(posedge CLK) begin
    Dd1_2 <= Dd1_1;
    DT0   <= DT1;
  end

  always @(negedge CLK) begin
    Dd0_2 <= Dd0_1;
    DT1   <= Ttx1;
  end

  assign Q0 = (CLK) ? Dd0_2 : Dd1_2;
  assign Q1 = (TXCLK_POL == 1'b0) ? DT0 : DT1;

endmodule  // ODDR (ddr output)

/**
 * Output serialiser, 4:1 .
 */
module OSER4 (
    Q0,
    Q1,
    D0,
    D1,
    D2,
    D3,
    TX0,
    TX1,
    PCLK,
    FCLK,
    RESET
);

  parameter GSREN = "false";  //"true"; "false"
  parameter LSREN = "true";  //"true"; "false"
  parameter HWL = "false";  //"true"; "false"
  parameter TXCLK_POL = 1'b0;  //1'b0:Rising edge output; 1'b1:Falling edge output

  input D3, D2, D1, D0;
  input TX1, TX0;
  input PCLK, FCLK, RESET;
  output Q0, Q1;

  reg [3:0] Dd1, Dd2, Dd3;
  reg [1:0] Ttx1, Ttx2, Ttx3;
  reg rstn_dsel, dsel, d_up0, d_up1;
  wire d_en0, d_en1;
  reg Qq_n, Q_data_n, Qq_p, Q_data_p;
  wire grstn, lrstn;

  initial begin
    dsel = 1'b0;
  end

  assign grstn = GSREN == "true" ? GSR.GSRO : 1'b1;
  assign lrstn = LSREN == "true" ? (~RESET) : 1'b1;

  always @(posedge PCLK or negedge grstn or negedge lrstn) begin
    if (!grstn) begin
      Dd1  <= 4'b0;
      Ttx1 <= 2'b0;
    end else if (!lrstn) begin
      Dd1  <= 4'b0;
      Ttx1 <= 2'b0;
    end else begin
      Dd1  <= {D3, D2, D1, D0};
      Ttx1 <= {TX1, TX0};
    end
  end

  always @(posedge FCLK or negedge grstn or negedge lrstn) begin
    if (!grstn) begin
      rstn_dsel <= 1'b0;
    end else if (!lrstn) begin
      rstn_dsel <= 1'b0;
    end else begin
      rstn_dsel <= 1'b1;
    end
  end

  always @(posedge FCLK or negedge rstn_dsel) begin
    if (!rstn_dsel) begin
      dsel <= 1'b0;
    end else begin
      dsel <= ~dsel;
    end
  end

  assign d_en0 = ~dsel;
  assign d_en1 = (HWL == "true") ? (~dsel) : dsel;

  always @(posedge FCLK or negedge rstn_dsel) begin
    if (!rstn_dsel) begin
      d_up0 <= 1'b0;
      d_up1 <= 1'b0;
    end else begin
      if (d_en0) begin
        d_up0 <= 1'b1;
      end else begin
        d_up0 <= 1'b0;
      end

      if (d_en1) begin
        d_up1 <= 1'b1;
      end else begin
        d_up1 <= 1'b0;
      end
    end
  end

  always @(posedge FCLK or negedge grstn or negedge lrstn) begin
    if (!grstn) begin
      Dd2  <= 4'b0;
      Ttx2 <= 2'b0;
    end else if (!lrstn) begin
      Dd2  <= 4'b0;
      Ttx2 <= 2'b0;
    end else begin
      if (d_up0) begin
        Dd2  <= Dd1;
        Ttx2 <= Ttx1;
      end else begin
        Dd2  <= Dd2;
        Ttx2 <= Ttx2;
      end
    end
  end

  always @(posedge FCLK or negedge grstn or negedge lrstn) begin
    if (!grstn) begin
      Dd3  <= 4'b0;
      Ttx3 <= 2'b0;
    end else if (!lrstn) begin
      Dd3  <= 4'b0;
      Ttx3 <= 2'b0;
    end else begin
      if (d_up1) begin
        Dd3  <= Dd2;
        Ttx3 <= Ttx2;
      end else begin
        Dd3[0]  <= Dd3[2];
        Dd3[2]  <= 1'b0;
        Dd3[1]  <= Dd3[3];
        Dd3[3]  <= 1'b0;
        Ttx3[0] <= Ttx3[1];
        Ttx3[1] <= 1'b0;
      end
    end
  end

  always @(negedge FCLK or negedge grstn or negedge lrstn) begin
    if (!grstn) begin
      Qq_n <= 1'b0;
      Q_data_n <= 1'b0;
    end else if (!lrstn) begin
      Qq_n <= 1'b0;
      Q_data_n <= 1'b0;
    end else begin
      Qq_n <= Dd3[0];
      Q_data_n <= Ttx3[0];
    end
  end

  always @(posedge FCLK or negedge grstn or negedge lrstn) begin
    if (!grstn) begin
      Qq_p <= 1'b0;
    end else if (!lrstn) begin
      Qq_p <= 1'b0;
    end else begin
      Qq_p <= Dd3[1];
    end
  end

  always @(posedge FCLK or negedge grstn or negedge lrstn) begin
    if (!grstn) begin
      Q_data_p <= 1'b0;
    end else if (!lrstn) begin
      Q_data_p <= 1'b0;
    end else begin
      Q_data_p <= Q_data_n;
    end
  end

  assign Q0 = FCLK ? Qq_n : Qq_p;
  assign Q1 = (TXCLK_POL == 1'b0) ? Q_data_p : Q_data_n;


endmodule  // OSER4 (4 to 1 serializer)
