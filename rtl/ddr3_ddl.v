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
module ddr3_ddl (  /*AUTOARG*/
    clock,
    reset,

    ctl_req_i,
    ctl_rdy_o,
    ctl_ref_o,
    ctl_cmd_i,
    ctl_ba_i,
    ctl_adr_i,

    mem_wvalid_i,
    mem_wready_o,
    mem_wlast_i,
    mem_wrmask_i,
    mem_wrdata_i,

    mem_rvalid_o,
    mem_rready_i,
    mem_rlast_o,
    mem_rddata_o,

    dfi_rst_no,
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
    dfi_data_i,
    dfi_rddata_dnv_i
);

  //
  //  DDL Settings
  ///

  // -- DDR3 SDRAM Timings and Parameters -- //

  parameter DDR_FREQ_MHZ = 100;
  localparam TCK = 1000 / DDR_FREQ_MHZ;  // in ns (nanoseconds)

  // Data-path and address settings
  parameter DDR_COL_BITS = 10;
  localparam CSB = DDR_COL_BITS - 1;
  parameter DDR_ROW_BITS = 13;
  localparam RSB = DDR_ROW_BITS - 1;

  parameter DFI_DATA_WIDTH = 32;
  localparam MSB = DFI_DATA_WIDTH - 1;

  parameter DFI_DQM_WIDTH = DFI_DATA_WIDTH / 8;
  localparam SSB = DFI_DQM_WIDTH - 1;


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
  parameter DDR_TWR = 15;  // post-WRITE recovery time, in ns
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


  localparam WR_CYCLES = (DDR_TWR + TCK - 1) / TCK;
  localparam CWL_CYCLES = 6;  // DLL=off


  // Mode Registers:
  localparam PPD = 1'b0;  // Slow exit (PRE PD), for DLL=off
  localparam [2:0] WRC = WR_CYCLES == 6 ? 3'b010 : (WR_CYCLES == 5 ? 3'b001 : 3'b000);
  localparam DLLR = 1'b0;  // No DLL reset, for DLL=off
  localparam [3:0] CAS = 4'b0100;  // CL=6 for DLL=off
  localparam [1:0] BLEN = 2'b00;  // BL8
  localparam [12:0] MR0 = {PPD, WRC, DLLR, 1'b0, CAS[3:1], 1'b0, CAS[0], BLEN};

  localparam QOFF = 1'b0;
  localparam DLLE = 1'b1;  // DLL=off
  localparam [1:0] DIC = 2'b00;  // Driver Impedance Control
  localparam [2:0] RTT = 3'b000;  // Nom, nom, nom, ...
  localparam [1:0] AL = 2'b00;  // Additive Latency disabled; todo: CL-1 ??
  localparam WLE = 1'b0;  // Write Leveling Enable is off
  localparam TDQS = 1'b0;  // Only applies to x8 memory
  localparam [12:0] MR1 = {
    QOFF, TDQS, 1'b0, RTT[2], 1'b0, WLE, RTT[1], DIC[1], AL, RTT[0], DIC[0], DLLE
  };

  localparam [1:0] RTTWR = 2'b00;  // Dynamic ODT off
  localparam SRT = 1'b0;  // Normal temperature for self-refresh
  localparam ASR = 1'b0;  // Manual self-refresh reference
  localparam [2:0] CWL = DLLE == 1'b0 ? CWL_CYCLES - 5 : 3'b001;  // DLL=off, so CWL=6
  localparam [2:0] PASR = 3'b000;  // Full Array
  localparam [12:0] MR2 = {2'b00, RTTWR, 1'b0, SRT, ASR, CWL, PASR};

  localparam [12:0] MR3 = {13'h0000};


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
  output ctl_rdy_o;
  output ctl_ref_o;  // refresh-request
  input [2:0] ctl_cmd_i;
  input [2:0] ctl_ba_i;
  input [RSB:0] ctl_adr_i;


  // AXI4-ish write and read ports (in order to de-/en- queue data from/to FIFOs,
  // efficiently)
  input mem_wvalid_i;
  output mem_wready_o;
  input mem_wlast_i;  // todo: a good idea ??
  input [SSB:0] mem_wrmask_i;
  input [MSB:0] mem_wrdata_i;

  output mem_rvalid_o;
  input mem_rready_i;
  output mem_rlast_o;  // todo: a good idea ??
  output [MSB:0] mem_rddata_o;

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
  input [1:0] dfi_rddata_dnv_i;  // ??


  // -- Constants -- //

  localparam TACTIVATE = 4;
  localparam TREFRESH = 16;
  localparam TPRECHARGE = 4;


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

  // But we always have CS# asserted, so:
  localparam CMD_NOOP = 3'b111;
  localparam CMD_ZQCL = 3'b110;
  localparam CMD_READ = 3'b101;
  localparam CMD_WRIT = 3'b100;
  localparam CMD_ACTV = 3'b011;
  localparam CMD_PREC = 3'b010;
  localparam CMD_REFR = 3'b001;
  localparam CMD_MODE = 3'b000;

  // REFRESH settings
  localparam CREFI = (DDR_TREFI - 1) / TCK;  // cycles(tREFI) - 1
  localparam RFC_BITS = $clog2(CREFI);
  localparam RFCSB = RFC_BITS - 1;
  localparam [RFCSB:0] RFC_ZERO = {RFC_BITS{1'b0}};


  reg [RFCSB:0] refresh_counter;
  reg [2:0] refresh_pending;

  reg ready;

  reg [3:0] cmd_prev_q, cmd_curr_q;
  wire [3:0] cmd_next_w;


  assign ctl_rdy_o = ready;

  assign dfi_rst_no = 1'bx;  // toods ...


  // -- Connect FIFO's to the DDR IOB's -- //

  assign dfi_mask_o = mem_wrmask_i;
  assign dfi_data_o = mem_wrdata_i;

  assign mem_rvalid_o = dfi_valid_i;
  assign mem_rddata_o = dfi_data_i;


  // -- Chip Enable -- //

  reg cke_q;

  assign dfi_cke_o = cke_q;

  always @(posedge clock) begin
    if (reset) begin
      cke_q <= 1'b0;
    end else begin
      if (dfi_rst_no == 1'b0) begin
        cke_q <= 1'b1;  // toods
      end
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
      ready <= 1'b0;
    end else begin
      case (state)
        ST_IDLE: begin
          ready <= 1'b1;
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
  reg ddr_awake;

  always @(posedge clock) begin
    if (reset) begin
      sinit <= SI_REST;
      ddr_awake <= 1'b0;
    end else begin
      if (refresh_pending != 3'b000) begin
        ddr_awake <= 1'b1;
      end
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
          if (ddr_awake) begin
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
          if (dfi_rst_no != 1'b0) begin
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

  wire refresh_issued = ctl_cmd_i == CMD_REFR && ctl_rdy_o;

  always @(posedge clock) begin
    if (reset) begin
      refresh_counter <= CREFI;
      refresh_pending <= 3'd0;
    end else begin
      if (refresh_counter == RFC_ZERO) begin
        refresh_counter <= CREFI;

        // REFRESH completed?
        if (refresh_issued) begin
          refresh_pending <= refresh_pending;
        end else begin
          refresh_pending <= refresh_pending + 1;
        end
      end else begin
        refresh_counter <= refresh_counter - 1;

        // REFRESH completed?
        if (refresh_issued && refresh_pending != 3'd0) begin
          refresh_pending <= refresh_pending - 1;
        end else begin
          refresh_pending <= refresh_pending;
        end
      end
    end
  end


endmodule  // ddr3_ddl
