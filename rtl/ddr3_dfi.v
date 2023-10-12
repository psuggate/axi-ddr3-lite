`timescale 1ns / 100ps
/**
 * Converts simple memory-controller commands into DFI commands.
 * 
 * Notes:
 *  - assumes that the AXI4 interface converts write-data into 128-bit chunks,
 *    padding as required;
 *  - read data will also be a (continuous) stream of 128-bit chunks, so the
 *    AXI4 interface will have to drop any (unwanted) trailing data, if not
 *    required;
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
  input [3:0] command_i;
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

  // DDR3 PHY Interface (-ish)
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


endmodule  // ddr3_dfi
