`timescale 1ns / 100ps
/**
 * Converts simple memory-controller commands into DFI commands.
 * 
 * Notes:
 *  - handles device- and mode- specific timings;
 *  - no flow-control, nor buffering -- as this belongs at higher layers;
 *  - assumes that the memory controller and the AXI4 bus are within the same
 *    clock-domain;
 * 
 * Copyright 2023, Patrick Suggate.
 * 
 */
module ddr3_dfi (  /*AUTOARG*/);

  parameter DDR_FREQ_MHZ = 100;
  parameter DDR_WR_LATENCY = 6;
  parameter DDR_RD_LATENCY = 5;

  // A DDR3 burst has length of 8 transfers (DDR), so four clock/memory cycles
  localparam DDR_BURST_LEN = 4;

  localparam DDR_BANK_BITS = 3;
  localparam BSB = DDR_BANK_BITS - 1;
  parameter DDR_COL_BITS = 9;
  localparam CSB = DDR_COL_BITS - 1;
  parameter DDR_ROW_BITS = 15;
  localparam RSB = DDR_ROW_BITS - 1;

  parameter DDR_DATA_WIDTH = 32;
  localparam MSB = DDR_DATA_WIDTH - 1;

  parameter DDR_DQM_WIDTH = DDR_DATA_WIDTH / 8;
  localparam SSB = DDR_DQM_WIDTH - 1;


  input clock;
  input reset;

  // From/to DDR3 Controller
  input enable_i;
  input request_i;
  input [3:0] command_i;
  input autopre_i;
  output accept_o;
  input [BSB:0] bank_i;
  input [RSB:0] addr_i;

  // AXI4-ish write and read ports (in order to de-/en- queue data from/to FIFOs,
  // efficiently)
  input wvalid_i;
  output wready_o;
  input wrlast_i;  // todo: a good idea ??
  input [SSB:0] wrmask_i;
  input [MSB:0] wrdata_i;

  output rvalid_o;
  input rready_i;
  output rdlast_o;  // todo: a good idea ??
  output [MSB:0] rddata_o;

  // (Pseudo-) DDR3 PHY Interface (-ish)
  output dfi_cke_o;
  output dfi_reset_n_o;
  output dfi_cs_n_o;
  output dfi_ras_n_o;
  output dfi_cas_n_o;
  output dfi_we_n_o;
  output dfi_odt_o;
  output [BSB:0] dfi_bank_o;
  output [RSB:0] dfi_addr_o;

  output dfi_wren_o;
  output [SSB:0] dfi_mask_o;
  output [MSB:0] dfi_data_o;

  output dfi_rden_o;
  input dfi_valid_i;
  input [MSB:0] dfi_data_i;
  input [1:0] dfi_rddata_dnv_i;  // ??


  // -- Constants -- //

  localparam TACTIVATE = 4;
  localparam TREFRESH = 16;
  localparam TPRECHARGE = 4;

  // localparam REFRESH_CYCLES =

  // DDR3 Commands: {CS#, RAS#, CAS#, WE#}
  localparam DDR3_NOOP = 4'b0111;
  localparam DDR3_ZQCL = 4'b0110;
  localparam DDR3_READ = 4'b0101;
  localparam DDR3_WRIT = 4'b0100;
  localparam DDR3_ACTV = 4'b0011;
  localparam DDR3_PREC = 4'b0010;
  localparam DDR3_REFR = 4'b0001;
  localparam DDR3_MODE = 4'b0000;

  localparam DDR3_PREA = 4'bxxxx;


  reg [3:0] cmd_prev_q, cmd_curr_q;
  wire [3:0] cmd_next_w;


  // -- Connect FIFO's to the DDR IOB's -- //

  assign dfi_mask_o = wrmask_i;
  assign dfi_data_o = wrdata_i;

  assign rvalid_o   = dfi_valid_i;
  assign rddata_o   = dfi_data_i;


  // -- Chip Enable -- //

  reg cke_q;

  assign dfi_cke_o = cke_q;

  always @(posedge clock) begin
    if (reset) begin
      cke_q <= 1'b0;
    end else begin
      cke_q <= enable_i;
    end
  end


  // todo:
  //  - (row) activate delays
  //  - read sequencing
  //  - write sequencing
  //  - read -> read sequencing
  //  - read -> write sequencing
  //  - write -> write sequencing
  //  - write -> read sequencing
  //  - (row) precharge delays
  //  - refresh delays
  //  - precharge -> refresh sequencing
  //  - mode-register read & write sequencing

  localparam ST_IDLE = 4'b0000;
  localparam ST_ACTV = 4'b0001;
  localparam ST_READ = 4'b0010;
  localparam ST_WRIT = 4'b0011;
  localparam ST_PREC = 4'b1101;
  localparam ST_PREA = 4'b1110;
  localparam ST_REFR = 4'b1111;

  reg [3:0] state;

  // todo: needs one of these per-bank ??
  always @(posedge clock) begin
    if (reset) begin
      state <= ST_IDLE;
    end else begin
      case (state)
        ST_IDLE: begin
          // transitions:
          //  - REFR
          //  - ACTV
        end

        ST_ACTV: begin
          // Activated banks & rows
        end

        ST_PREC: begin
          // Wait until timer has elapsed
        end

        ST_PREA: begin
          // Wait until timer has elapsed
        end

        ST_REFR: begin
          // Wait until timer has elapsed
        end
      endcase
    end
  end


endmodule  // ddr3_dfi
