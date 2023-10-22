`timescale 1ns / 100ps
/**
 * DDR3 "Data-Link" layer converts simple memory-controller commands into DFI
 * commands.
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
module ddr3_ddl (
    clock,
    reset,

    ctl_req_i,  // Memory controller signals
    ctl_run_i,
    ctl_rdy_o,
    ctl_ref_o,
    ctl_cmd_i,
    ctl_ba_i,
    ctl_adr_i,

    mem_wvalid_i,  // WRITE data-path
    mem_wready_o,
    mem_wlast_i,
    mem_wrmask_i,
    mem_wrdata_i,

    mem_rvalid_o,  // READ data-path
    mem_rready_i,
    mem_rlast_o,
    mem_rddata_o,

    dfi_rst_no,  // DDL <-> PHY signals
    dfi_cke_o,
    dfi_cs_no,
    dfi_ras_no,
    dfi_cas_no,
    dfi_we_no,
    dfi_odt_o,
    dfi_bank_o,
    dfi_addr_o,
    dfi_wren_o,
    dfi_mask_o,
    dfi_data_o,
    dfi_rden_o,
    dfi_valid_i,
    dfi_data_i
);

  //
  //  DDL Settings
  ///

  // -- DDR3 SDRAM Timings and Parameters -- //

  parameter DDR_FREQ_MHZ = 100;
  `include "ddr3_settings.vh"

  // Data-path and address settings
  parameter DDR_COL_BITS = 10;
  localparam CSB = DDR_COL_BITS - 1;
  parameter DDR_ROW_BITS = 13;
  localparam RSB = DDR_ROW_BITS - 1;

  parameter DFI_DATA_WIDTH = 32;
  localparam MSB = DFI_DATA_WIDTH - 1;

  parameter DFI_MASK_WIDTH = DFI_DATA_WIDTH / 8;
  localparam SSB = DFI_MASK_WIDTH - 1;


  // -- PHY Settings -- //

  // Note: these latencies are due to the registers and IOBs in the PHY for the
  //   commands, addresses, and the data-paths.
  parameter PHY_WR_LATENCY = 2;
  parameter PHY_RD_LATENCY = 5;

  // A DDR3 burst has length of 8 transfers (DDR), so four clock/memory cycles
  localparam PHY_BURST_LEN = 4;


  input clock;
  input reset;

  // From/to DDR3 Controller
  // Note: all state-transitions are gated by the 'ctl_rdy_o' signal
  input ctl_req_i;
  input ctl_run_i;
  output ctl_rdy_o;
  output ctl_ref_o;  // refresh-request
  input [2:0] ctl_cmd_i;
  input [2:0] ctl_ba_i;
  input [RSB:0] ctl_adr_i;


  // AXI4-ish write and read ports (in order to de-/en- queue data from/to FIFOs,
  // efficiently)
  input mem_wvalid_i;  // Write port
  output mem_wready_o;
  input mem_wlast_i;  // todo: a good idea ??
  input [SSB:0] mem_wrmask_i;
  input [MSB:0] mem_wrdata_i;

  output mem_rvalid_o;  // Read port
  input mem_rready_i;
  output mem_rlast_o;  // todo: a good idea ??
  output [MSB:0] mem_rddata_o;

  output byp_rvalid_o;  // Read fast-path port
  input byp_rready_i;
  output byp_rlast_o;  // todo: a good idea ??
  output [MSB:0] byp_rddata_o;

  // (Pseudo-) DDR3 PHY Interface (-ish)
  output dfi_cke_o;
  output dfi_rst_no;
  output dfi_cs_no;
  output dfi_ras_no;
  output dfi_cas_no;
  output dfi_we_no;
  output dfi_odt_o;
  output [2:0] dfi_bank_o;
  output [RSB:0] dfi_addr_o;

  output dfi_wren_o;
  output [SSB:0] dfi_mask_o;
  output [MSB:0] dfi_data_o;

  output dfi_rden_o;
  input dfi_valid_i;
  input [MSB:0] dfi_data_i;


  // -- Constants -- //

  localparam TACTIVATE = 4;
  localparam TREFRESH = 16;
  localparam TPRECHARGE = 4;


  // -- DDR Command Dispatch Delay Constraints -- //

  // Minimum cycles between same bank ACT -> ACT
  localparam CYCLES_ACT_TO_ACT = (DDR_TRC + TCK - 1) / TCK;  // both of these must
  localparam CYCLES_ACT_TO_PRE = (DDR_TRAS + TCK - 1) / TCK;  // be satisfied

  // Minimum cycles between ACT and REF
  localparam CYCLES_ACT_TO_REF = CYCLES_ACT_TO_ACT;  // b/c ACT -> PRE -> REF

  // Minimum cycles for a self-REF to complete, before ACT (RAS# := 0)
  localparam CYCLES_REF_TO_ACT = (DDR_TRFC + TCK - 1) / TCK;

  // Minimum of 2 cycles PRE -> ACT (same bank, defaults, DLL=off)
  localparam CYCLES_PRE_TO_ACT = (DDR_TRP + TCK - 1) / TCK;

  // RAS# -> CAS# is the same as CAS# -> DQ (valid)
  localparam CYCLES_ACT_TO__RD = DDR_CL;
  localparam CYCLES_ACT_TO__WR = DDR_CWL;

  // RD->RD & WR->WR are spaced by at least CAS# -> CAS# delays (of 4 cycles)
  localparam CYCLES__RD_TO__RD = DDR_CCCD;  // 4 cycles
  localparam CYCLES__WR_TO__WR = DDR_CCCD;  // 4 cycles

  // Minimum of 6 cycles RD -> WR (with default settings, DLL=off)
  localparam CYCLES__RD_TO__WR = DDR_CL + DDR_CCCD + 2 - DDR_CWL;

  // Minimum cycles between issuing WR and RD (same bank)
  localparam CYCLES__WR_TO__RD = DDR_CWL + 4 + DDR_CWTR;  // 14 cycles, BL8, DLL=off

  // Minimum cycles between issuing WR and PRE (same bank)
  localparam CYCLES__WR_TO_PRE = DDR_CWL + 4 + (DDR_TWR + TCK - 1) / TCK;  // 12 cycles

  reg ready;
reg cmd_done = 1'b0; // toodoos

  reg [3:0] cmd_prev_q, cmd_curr_q;
  wire [3:0] cmd_next_w;


  assign ctl_rdy_o = ready;

  assign dfi_rst_no = 1'bx;  // toods ...
  assign dfi_cke_o = cke_q;
  assign dfi_cs_no = cs_nq;
  assign dfi_odt_o = 1'b0;


  // -- Connect FIFO's to the DDR IOB's -- //

  assign dfi_mask_o = mem_wrmask_i;
  assign dfi_data_o = mem_wrdata_i;

  assign mem_rvalid_o = dfi_valid_i;
  assign mem_rddata_o = dfi_data_i;


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

  localparam ST_INIT = 4'b0000;
  localparam ST_IDLE = 4'b0001;
  localparam ST_ACTV = 4'b0010;
  localparam ST_READ = 4'b0100;
  localparam ST_WRIT = 4'b0101;
  localparam ST_PREC = 4'b1101;
  localparam ST_PREA = 4'b1110;
  localparam ST_REFR = 4'b1111;

  reg [3:0] state;

  // todo: needs one of these per-bank ??
  always @(posedge clock) begin
    if (reset) begin
      state <= ST_INIT;
      ready <= 1'b0;
    end else begin
      case (state)
        ST_INIT: begin
          // Wait for the (first steps of the) initialisation to finish
          if (ctl_run_i) begin
            state <= ST_IDLE;
          end
        end

        ST_IDLE: begin
          if (ctl_req_i) begin
            // Possible transitions:
            //  - ACTV  --  precedes RD/WR commands
            //  - PREC  --  precedes REFR (but we don't keep pages ACTV + IDLE) ??
            //  - REFR  --  REFRESH periodically
            //  - MODE  --  during initialisation
            //  - ZQCL  --  as the last step during initialisation
            case (ctl_cmd_i)
              CMD_ACTV: begin
                // Row-ACTIVATE, which precedes RD/WR
                // Default activation time is 6 cycles (DLL=off)
                state <= ST_ACTV;
              end

              CMD_PREC: begin
                // Bank (and row) PRECHARGE
                // Note: usually a PRECHARGE ALL command
                // Default is 2 cycles (DLL=off)
                state <= ST_PREC;
              end

              CMD_REFR: begin
                // REFRESH the SDRAM contents
                // Defaults to 11 cycles, at 100 MHz (DLL=off)
                state <= ST_REFR;
              end

              CMD_MODE: begin
                // Set MODE register
                state <= ST_MODE;
              end

              CMD_ZQCL: begin
                // Impedance calibration
                // Defaults to 512 cycles, with DLL=off
                state <= ST_ZQCL;
              end

              CMD_NOOP: begin
                // Ignore these, other than noting that the memory-contoller is
                // being a bit weird ...
                $display("%10t: DDL: Unnecessary NOP request", $time);
                ready <= 1'b1;
              end

              default: begin
                $error("%10t: DDL: Invalid command: 0x%1x", $time, ctl_cmd_i);
                state <= ST_INIT;
              end
            endcase
          end
        end

        ST_ACTV: begin
          // ACTIVATE a row/page within a bank
          // Possible transitions:
          //  - ACTV  --  allow back-to-back ACTIVATIONs ??
          //  - PREC  --  weird, but legit. ??
          //  - READ
          //  - WRIT
          if (cmd_done && ctl_req_i) begin
            case (ctl_cmd_i)
              CMD_READ: begin
              end

              CMD_WRIT: begin
              end

              default: begin
              end
            endcase
          end else if (cmd_done) begin
            // No command, so return to IDLE
            $display("%10t: DDL: ACTV -> IDLE -- undesirable?", $time);
            state <= ST_IDLE;
          end
        end

        ST_PREC: begin
          // Wait until timer has elapsed, for a PRECHARGE
          // Possible transitions:
          //  - ACTV
          //  - REFR
          //  - IDLE
          //  - MODE  --  weird, but legit. ??
          // plus unsupported stuff, like power-down.
          if (cmd_done && ctl_req_i) begin
            case (ctl_cmd_i)
              CMD_ACTV: begin
              end

              default: begin
              end
            endcase
          end else if (cmd_done) begin
            state <= ST_IDLE;
          end
        end

        ST_REFR: begin
          // Wait until timer has elapsed
          // Possible transitions:
          //  - ACTV
          //  - IDLE
          //  - MODE  --  weird, but legit. ??
          if (cmd_done && ctl_req_i) begin
            case (ctl_cmd_i)
              CMD_ACTV: begin
                state <= ST_ACTV;
                ready <= 1'b1;
              end

              default: begin
              end
            endcase
          end else if (cmd_done) begin
            state <= ST_IDLE;
          end
        end

        ST_READ: begin
          if (ctl_cmd_i == CMD_READ) begin
            // RD -> RD (default: 4 cycles)
          end else if (ctl_cmd_i == CMD_WRIT) begin
            // RD -> WR (default: 6 cycles)
          end else if (ctl_cmd_i == CMD_ACTV) begin
            // RD -> ACT (default: 4 cycles)
          end else begin
            // RD -> NOP
          end
        end

        ST_WRIT: begin
          if (ctl_cmd_i == CMD_WRIT) begin
            // WR -> WR (default: 4 cycles)
          end else if (ctl_cmd_i == CMD_READ) begin
            // WR -> RD (default: 14 + 4 cycles)
          end else if (ctl_cmd_i == CMD_ACTV) begin
            // WR -> ACT (default: 4 cycles, different bank)
            // WR -> PRE -> ACT (default: 16 cycles, same bank)
          end else begin
            // WR -> NOP
          end
        end

        default: begin
          $error("%10t: DDL: Unexpected state: 0x%02x", $time, state);
          state <= ST_INIT;
        end
      endcase
    end
  end


// -- Shift-Registers for Command Delays -- //

  shift_register #(
      .WIDTH(1),
      .DEPTH(16)
  ) rd_rd_srl_inst (
      .clock (clock),
      .wren_i(1'b1),
      .data_i(dfi_rden_i),
      .addr_i(rd_lat_q),
      .data_o(rd_en_w)
  );


endmodule  // ddr3_ddl
