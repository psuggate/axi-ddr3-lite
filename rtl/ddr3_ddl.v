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
module ddr3_ddl (  /*AUTOARG*/);

  //
  //  DDL Settings
  ///

  // -- DDR3 SDRAM Timings and Parameters -- //

  parameter DDR_FREQ_MHZ = 100;
  localparam TCK = 1000 / DDR_FREQ_MHZ;  // in ns (nanoseconds)

  // Minimum period in DLL=off mode is 8 ns (or, max freq of 125 MHz)
  parameter DDR_CL = 6;  // DLL=off mode settings
  parameter DDR_CWL = 6;

  // DDR reset, refresh, and initialisation parameters
  parameter DDR_TREFI = 7800;  // REFRESH-interval in ns, at normal temperatures
  parameter DDR_TRFC = 110;  // self-REFRESH duration, in ns, for 1Gb DDR3 SDRAM
  localparam DDR_TRESET = 200000;  // RESET# for 200 us after power-on
  localparam DDR_TWAKE = 500000;  // after RESET# deasserts, before first command
  localparam DDR_TCKE0 = 10;  // at least 10 ns between CKE := 0 and RESET# := 1
  localparam DDR_CCKE1 = 5;  // at least 5x cycles between CK valid and CKE := 1
  parameter DDR_TXPR = "todo";  // tXPR := max(tXS; 5x tCK)
  // parameter DDR_TRRD = 10; // tRRD := max(10ns; 4x tCK)

  // DDR3-800E (6-6-6) speed bin parameters (from pp. 157)
  parameter DDR_TAAMIN = 15;  // min time (ns) for internal-read -> data
  parameter DDR_TAAMAX = 20;  // max time (ns) for internal-read -> data
  parameter DDR_TWR = 15;  // post-WRITE, AUTO-PRECHARGE time, in ns
  parameter DDR_TRP = 15;  // min time (ns) for PRE command
  parameter DDR_TRCD = 15;  // min time (ns) for ACT -> internal rd/wr
  parameter DDR_TRC = 52.5;  // min ACT -> {ACT, REF} command period
  parameter DDR_TRAS = 37.5;  // min ACT -> PRE command period

  // From pp. 169
  parameter DDR_CDLLK = 512;  // number of cycles for DLL lock
  parameter DDR_CZQINIT = 512;  // cycles for ZCQL, or 640 ns (if greater)
  parameter DDR_CRTP = 4;  // cycles for internal READ -> PRE
  parameter DDR_CWTR = 4;  // cycles for internal WRITE -> internal READ
  parameter DDR_CMRD = 4;  // cycles for Mode Reg. Set command
  parameter DDR_CMOD = 12;  // cycles for Mode Reg. Set update
  parameter DDR_CCCD = 4;  // CAS# -> CAS# command delay (cycles)
  parameter DDR_CRRD = 4;  // min ACT -> ACT


  parameter DDR_COL_BITS = 10;
  localparam CSB = DDR_COL_BITS - 1;
  parameter DDR_ROW_BITS = 13;
  localparam RSB = DDR_ROW_BITS - 1;

  parameter DDR_DATA_WIDTH = 32;
  localparam MSB = DDR_DATA_WIDTH - 1;

  parameter DDR_DQM_WIDTH = DDR_DATA_WIDTH / 8;
  localparam SSB = DDR_DQM_WIDTH - 1;


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
  output ctl_rdy_o;
  output ctl_rfc_o;

  input enable_i;
  input request_i;
  input [3:0] command_i;
  input autopre_i;
  output accept_o;
  input [2:0] bank_i;
  input [RSB:0] addr_i;


// DDR Data-Layer control signals
// Note: all state-transitions are gated by the 'ddl_rdy_i' signal
input ddl_req_i;
output ddl_rdy_o;
output ddl_ref_o; // refresh-request
input [2:0] ddl_cmd_i;
input [2:0] ddl_ba_i;
input [RSB:0] ddl_adr_i;
// input [SSB:0] ddl_stb_i;
// input [MSB:0] ddl_dat_i;
// output [MSB:0] ddl_dat_o;


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
  output [2:0] dfi_bank_o;
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

  // REFRESH settings
  localparam CREFI = (DDR_TREFI - 1) / TCK;  // cycles(TREFI) - 1
  localparam CBITS = $clog2(CREFI);
  localparam CSB = CBITS - 1;
  localparam [CSB:0] CZERO = {CBITS{1'b0}};


  // -- DDR Command Dispatch Periods -- //

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


  // -- Initialisation State Machine -- //

  // todo:
  //  - CKE: 0 -> 1 in 500 us
  //  - 

  localparam [3:0] SI_REST = 4'b0000;
  localparam [3:0] SI_CKE1 = 4'b0001;
  localparam [3:0] SI_MODE = 4'b0010;
  localparam [3:0] SI_ZQCL = 4'b0100;
  localparam [3:0] SI_DONE = 4'b1000;

  reg [3:0] sinit;

  always @(posedge clock) begin
    if (reset) begin
      sinit <= SI_REST;
    end else begin
    end
  end


  // -- Mode-Setting State Machine -- //

  // todo: one-hot encoding ??
  localparam [3:0] MR_INIT = 4'b0000;
  localparam [3:0] MR_MODE = 4'b0001;
  localparam [3:0] MR_DONE = 4'b1000;

  reg [3:0] smode;

  always @(posedge clock) begin
    if (reset) begin
      smode <= MR_INIT;
    end else begin
      // Set the (post-RESET#) operating-mode for the device
      case (smode)
        MR_INIT: begin
          // Wait until the memory has woken up, then start setting the mode
          // registers
          if (ddr_rdy_i) begin
            smode <= MR_MODE;  // Set the mode registers
          end else begin
            smode <= smode;
          end
        end

        MR_MODE: begin
          // Step through each mode register: MR1 -> MR2 -> MR0
          smode <= MR_DONE;
        end

        MR_DONE: begin
          if (ddr_rst_no != 1'b0) begin
            // Stay "done" until reset
            smode <= smode;
          end else begin
            // We're now officially "not done"
            smode <= MR_INIT;
          end
        end

        default: begin
          $error("How !?");
          $fatal;
          smode <= MR_INIT;
        end
      endcase
    end
  end


  // -- Refresh Counter -- //

  always @(posedge clock) begin
    if (reset) begin
      refresh_counter <= CREFI;
      refresh_pending <= 3'd0;
    end else begin
      if (refresh_counter == CZERO) begin
        refresh_counter <= CREFI;

        // REFRESH completed?
        if (ddr_rfc_i) begin
          refresh_pending <= refresh_pending;
        end else begin
          refresh_pending <= refresh_pending + 1;
        end
      end else begin
        refresh_counter <= refresh_counter - 1;

        // REFRESH completed?
        if (ddr_rfc_i) begin
          refresh_pending <= refresh_pending - 1;
        end else begin
          refresh_pending <= refresh_pending;
        end
      end
    end
  end


endmodule  // ddr3_ddl
