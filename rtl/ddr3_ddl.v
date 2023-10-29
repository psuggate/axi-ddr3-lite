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

    ddr_cke_i,
    ddr_cs_ni,

    ctl_run_o,  // Memory controller signals
    ctl_req_i,
    ctl_seq_i,
    ctl_rdy_o,
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

    dfi_ras_no,  // DDL <-> PHY signals
    dfi_cas_no,
    dfi_we_no,
    dfi_odt_o,
    dfi_bank_o,
    dfi_addr_o,
    dfi_wstb_o,
    dfi_wren_o,
    dfi_mask_o,
    dfi_data_o,
    dfi_rden_o,
    dfi_rvld_i,
    dfi_data_i
);

  //
  //  DDL Settings
  ///

  // -- DDR3 SDRAM Timings and Parameters -- //

  parameter DDR_FREQ_MHZ = 100;
  `include "ddr3_settings.vh"

  // Data-path and address settings
  parameter DDR_ROW_BITS = 13;
  localparam RSB = DDR_ROW_BITS - 1;

  parameter DDR_COL_BITS = 10;
  localparam CSB = DDR_COL_BITS - 1;

  parameter DFI_DQ_WIDTH = 32;
  localparam MSB = DFI_DQ_WIDTH - 1;

  parameter DFI_DM_WIDTH = DFI_DQ_WIDTH / 8;
  localparam SSB = DFI_DM_WIDTH - 1;


  // -- PHY Settings -- //

  // Note: these latencies are due to the registers and IOBs in the PHY for the
  //   commands, addresses, and the data-paths.
  parameter PHY_WR_LATENCY = 1;
  parameter PHY_RD_LATENCY = 1;

  // A DDR3 burst has length of 8 transfers (DDR), so four clock/memory cycles
  // todo: ...
  localparam PHY_BURST_LEN = 4;


  input clock;
  input reset;

  input ddr_cke_i;
  input ddr_cs_ni;

  // From/to DDR3 Controller
  // Note: all state-transitions are gated by the 'ctl_rdy_o' signal
  output ctl_run_o;
  input ctl_req_i;
  input ctl_seq_i;  // Burst-sequence indicator
  output ctl_rdy_o;
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
  output dfi_ras_no;
  output dfi_cas_no;
  output dfi_we_no;
  output dfi_odt_o;
  output [2:0] dfi_bank_o;
  output [RSB:0] dfi_addr_o;

  output dfi_wstb_o;
  output dfi_wren_o;
  output [SSB:0] dfi_mask_o;
  output [MSB:0] dfi_data_o;

  output dfi_rden_o;
  input dfi_rvld_i;
  input [MSB:0] dfi_data_i;


  // -- Constants -- //

  localparam DELAY = CYCLES_CKE_TO_CMD + 1;
  localparam DSB = DELAY - 1;
  localparam DZERO = {DELAY{1'b0}};
  localparam DINIT = 1 << (CYCLES_CKE_TO_CMD - 1);

  localparam CBITS = $clog2(DDR_CZQINIT);
  localparam XSB = CBITS - 1;
  localparam CZERO = {CBITS{1'b0}};

  localparam WDLYS = 4;  // BL8 = 4 extra delays
  localparam WSB = WDLYS - 1;

  // Delays for the various transitions
  localparam DELAY_CKE_TO_CMD = 1 << (CYCLES_CKE_TO_CMD - 1);
  localparam DELAY_MRD_TO_CMD = 1 << (CYCLES_MRD_TO_CMD - 1);
  localparam DELAY_PRE_TO_ACT = 1 << (CYCLES_PRE_TO_ACT - 1);
  localparam DELAY_ACT_TO_PRE = 1 << (CYCLES_ACT_TO_PRE - 1);
  localparam DELAY_REF_TO_ACT = 1 << (CYCLES_REF_TO_ACT - 2);
  localparam DELAY_ACT_TO_REF = 1 << (CYCLES_ACT_TO_REF - 1);

  localparam DELAY_ACT_TO_ACT_L = 1 << (CYCLES_ACT_TO_ACT - 1);
  localparam DELAY_ACT_TO_ACT_S = DDR_CRRD - 1;

  localparam DELAY_ACT_TO_R_W = 1 << (CYCLES_ACT_TO_R_W - 2);
  localparam DELAY__RD_TO__RD = 1 << (CYCLES__RD_TO__RD - 2);
  localparam DELAY__RD_TO__WR = 1 << (CYCLES__RD_TO__WR - 2);
  localparam DELAY__WR_TO__RD = 1 << (CYCLES__WR_TO__RD - 2);
  localparam DELAY__WR_TO__WR = 1 << (CYCLES__WR_TO__WR - 2);

  // todo: fix/finalise these timings ...
  localparam DELAY_RDA_TO_ACT = 1 << (CYCLES_RDA_TO_ACT - 1);
  localparam DELAY_WRA_TO_ACT = 1 << (CYCLES_WRA_TO_ACT - 1);


  reg [WSB:0] wr_delay, rd_delay;
  reg wr_strob, wr_ready, rd_ready;
  reg run_q, ready, busy;
  reg [2:0] cmd_q, ba_q;
  reg [RSB:0] adr_q;

  wire ctl_pre_w;


  // -- Connect to Upstream Controller & Data-paths -- //

  assign ctl_run_o = run_q;
  assign ctl_rdy_o = ready;
  assign ctl_pre_w = ctl_adr_i[10];

  assign mem_wready_o = wr_ready;


  // -- Connect FIFO's to the DDR IOB's -- //

  assign dfi_ras_no = cmd_q[2];
  assign dfi_cas_no = cmd_q[1];
  assign dfi_we_no = cmd_q[0];
  assign dfi_odt_o = 1'b0;
  assign dfi_bank_o = ba_q;
  assign dfi_addr_o = adr_q;
  assign dfi_wstb_o = wr_strob;
  assign dfi_wren_o = wr_ready;
  assign dfi_mask_o = mem_wrmask_i;
  assign dfi_data_o = mem_wrdata_i;
  assign dfi_rden_o = rd_ready;

  assign mem_rvalid_o = dfi_rvld_i;
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

  localparam ST_IDLE = 4'b0001;
  localparam ST_ACTV = 4'b0010;
  localparam ST_READ = 4'b0100;
  localparam ST_WRIT = 4'b0101;
  localparam ST_PREC = 4'b1101;
  localparam ST_PREA = 4'b1110;
  localparam ST_REFR = 4'b1111;
  localparam ST_ZQCL = 4'd11;
  localparam ST_MODE = 4'd12;

  reg  [  3:0] state;
  reg  [DSB:0] delay;
  reg  [XSB:0] count;
  wire [XSB:0] cnext;

  assign cnext = count - 1;

  // todo: needs one of these per-bank ??
  always @(posedge clock) begin
    if (reset || !ddr_cke_i) begin
      state <= ST_IDLE;
      ready <= 1'b0;
      busy  <= 1'b0;
      delay <= DINIT;
      run_q <= 1'b0;
      cmd_q <= CMD_NOOP;
      count <= CZERO;
    end else if (busy || !ready) begin
      delay <= {1'b0, delay[DSB:1]};
      cmd_q <= CMD_NOOP;
      if (busy) begin
        count <= cnext;
      end
      if (cnext == CZERO) begin
        busy  <= 1'b0;
        ready <= 1'b1;
      end else begin
        ready <= delay[0];
        run_q <= run_q | delay[0];
      end
    end else if (ctl_req_i && ready) begin
      ready <= 1'b0;
      cmd_q <= ctl_cmd_i;
      ba_q  <= ctl_ba_i;
      adr_q <= ctl_adr_i;  // todo: auto-precharge bit (A[10])

      case (state)
        ST_IDLE: begin
          // Possible transitions:
          //  - ACTV  --  precedes RD/WR commands
          //  - PREC  --  precedes REFR (but we don't keep pages ACTV + IDLE) ??
          //  - REFR  --  REFRESH periodically
          //  - MODE  --  during initialisation
          //  - ZQCL  --  as the last step during initialisation
          case (ctl_cmd_i)
            CMD_ACTV: begin
              // Row-ACTIVATE, which precedes RD/WR
              // Default activation time is 6 cycles (DLL=off) ??
              state <= ST_ACTV;
              delay <= DELAY_ACT_TO_R_W;
            end

            CMD_PREC: begin
              // Bank (and row) PRECHARGE
              // Note: usually a PRECHARGE ALL command
              // Default is 2 cycles (DLL=off)
              state <= ST_PREC;
              delay <= DELAY_PRE_TO_ACT;
            end

            CMD_REFR: begin
              // REFRESH the SDRAM contents
              // Defaults to 11 cycles, at 100 MHz (DLL=off)
              state <= ST_REFR;
              delay <= DELAY_REF_TO_ACT;
            end

            CMD_MODE: begin
              // Set MODE register
              state <= ST_MODE;
              delay <= DELAY_MRD_TO_CMD;
            end

            CMD_ZQCL: begin
              // Impedance calibration
              // Defaults to 512 cycles, with DLL=off
              state <= ST_ZQCL;
              count <= DDR_CZQINIT;
              busy  <= 1'b1;
            end

            CMD_NOOP: begin
              // Ignore these, other than noting that the memory-contoller is
              // being a bit weird ...
              $display("%10t: DDL: Unnecessary NOP request", $time);
              ready <= 1'b1;
            end

            default: begin
              $error("%10t: DDL: Invalid command: 0x%1x", $time, ctl_cmd_i);
              state <= ST_IDLE;
              busy  <= 1'b1;
              count <= DDR_CZQINIT;
            end
          endcase
        end

        ST_ACTV: begin
          // ACTIVATE a row/page within a bank
          // Possible transitions:
          //  - ACTV  --  allow back-to-back ACTIVATIONs ??
          //  - PREC  --  weird, but legit. ??
          //  - READ
          //  - WRIT
          case (ctl_cmd_i)
            CMD_READ: begin
              if (ctl_pre_w) begin
                state <= ST_IDLE;
                delay <= DELAY_RDA_TO_ACT;
              end else begin
                state <= ST_READ;
                delay <= DELAY__RD_TO__RD;
              end
            end

            CMD_WRIT: begin
              if (ctl_pre_w) begin
                state <= ST_IDLE;
                delay <= DELAY_WRA_TO_ACT;
              end else begin
                state <= ST_WRIT;
                delay <= DELAY__WR_TO__WR;
              end
            end

            CMD_ACTV: begin
              // todo: delay depends on if same bank, or different
              state <= ST_ACTV;
              delay <= DELAY_ACT_TO_ACT_S;
            end

            default: begin
              state <= 'bx;
              delay <= 'bx;
              $error("%10t: DDL: Unexpected command (0x%1x) in 'ST_ACTV'", $time, ctl_cmd_i);
              $fatal;
            end
          endcase
        end

        ST_READ: begin
          case (ctl_cmd_i)
            CMD_READ: begin
              if (ctl_pre_w) begin
                state <= ST_IDLE;
                delay <= DELAY_RDA_TO_ACT;
              end else begin
                // RD -> RD (default: 4 cycles)
                delay <= DELAY__RD_TO__RD;
              end
            end

            CMD_WRIT: begin
              // RD -> WR (default: 6 cycles)
              delay <= DELAY__RD_TO__WR;
              state <= ST_WRIT;
            end

            CMD_ACTV: begin
              // RD -> ACT (default: 4 cycles)
              delay <= 2;
              state <= ST_ACTV;
            end

            default: begin
              state <= 'bx;
              delay <= 'bx;
              $error("%10t: DDL: Unexpected command (0x%1x) in 'ST_READ'", $time, ctl_cmd_i);
              $fatal;
            end
          endcase
        end

        ST_WRIT: begin
          case (ctl_cmd_i)
            CMD_WRIT: begin
              if (ctl_pre_w) begin
                // WR + AUTO-PRECHARGE (default: 14 cycles)
                delay <= DELAY_WRA_TO_ACT;
                state <= ST_IDLE;
              end else begin
                // WR -> WR (default: 4 cycles)
                delay <= DELAY__WR_TO__WR;
              end
            end
            CMD_READ: begin
              // WR -> RD (default: 14 + 4 cycles)
              state <= ST_READ;
              delay <= DELAY__WR_TO__RD;
            end
            CMD_ACTV: begin
              // WR -> ACT (default: 4 cycles, different bank)
              // WR -> PRE -> ACT (default: 16 cycles, same bank)
              state <= ST_ACTV;
              delay <= 2;
            end
            default: begin
              state <= 'bx;
              delay <= 'bx;
              $error("%10t: DDL: Unexpected command (0x%1x) in 'ST_WRIT'", $time, ctl_cmd_i);
              $fatal;
            end
          endcase
        end

        ST_PREC: begin
          // Wait until timer has elapsed, for a PRECHARGE
          // Possible transitions:
          //  - ACTV
          //  - REFR
          //  - IDLE
          //  - MODE  --  weird, but legit. ??
          // plus unsupported stuff, like power-down.
          case (ctl_cmd_i)
            CMD_ACTV: begin
              state <= ST_ACTV;
              delay <= DELAY_PRE_TO_ACT;
            end

            CMD_REFR: begin
              state <= ST_REFR;
              delay <= 1 << PRE_CYCLES;
            end

            CMD_NOOP: begin
              state <= ST_IDLE;
              delay <= 1 << PRE_CYCLES;
            end

            default: begin
              state <= 'bx;
              delay <= 'bx;
              $error("%10t: DDL: Unexpected command (0x%1x) in 'ST_PREC'", $time, ctl_cmd_i);
              $fatal;
            end
          endcase
        end

        ST_REFR: begin
          // Wait until timer has elapsed
          // Possible transitions:
          //  - ACTV
          //  - IDLE
          //  - MODE  --  weird, but legit. ??
          //  - REFR  --  up to 16x REFR can be issued within 2*tREFI
          case (ctl_cmd_i)
            CMD_ACTV: begin
              state <= ST_ACTV;
              delay <= DELAY_ACT_TO_R_W;
            end

            CMD_REFR: begin
              $display("%10t: DDL: Back-to-back REFRESH commands issued", $time);
              delay <= DELAY_REF_TO_ACT;
            end

            default: begin
              state <= 'bx;
              delay <= 'bx;
              $error("%10t: DDL: Unexpected command (0x%1x) in 'ST_REFR'", $time, ctl_cmd_i);
              $fatal;
            end
          endcase
        end

        ST_MODE: begin
          case (ctl_cmd_i)
            CMD_MODE: begin
              delay <= DELAY_MRD_TO_CMD;
            end

            CMD_REFR: begin
              state <= ST_REFR;
              delay <= DELAY_REF_TO_ACT;
            end

            CMD_PREC: begin
              state <= ST_PREC;
              delay <= DELAY_PRE_TO_ACT;
            end

            CMD_ZQCL: begin
              state <= ST_ZQCL;
              busy  <= 1'b1;
              count <= DDR_CZQINIT;
            end

            default: begin
              state <= 'bx;
              delay <= 'bx;
              $error("%10t: DDL: Unexpected command (0x%1x) in 'ST_MODE'", $time, ctl_cmd_i);
              $fatal;
            end
          endcase
        end

        ST_ZQCL: begin
          state <= ST_IDLE;
          case (ctl_cmd_i)
            CMD_ACTV: begin
              state <= ST_ACTV;
              delay <= DELAY_ACT_TO_R_W;
            end

            CMD_REFR: begin
              state <= ST_REFR;
              delay <= DELAY_REF_TO_ACT;
            end

            CMD_PREC: begin
              state <= ST_PREC;
              delay <= DELAY_PRE_TO_ACT;
            end

            default: begin
              state <= 'bx;
              delay <= 'bx;
              $error("%10t: DDL: Unexpected command (0x%1x) in 'ST_ZQCL'", $time, ctl_cmd_i);
              $fatal;
            end
          endcase
        end

        default: begin
          $error("%10t: DDL: Unexpected state: 0x%02x", $time, state);
          state <= ST_IDLE;
        end
      endcase
    end else if (ready) begin
      // No follow-up command, so IDLE
      state <= ST_IDLE;
      cmd_q <= CMD_NOOP;
    end
  end


  // -- Write Data-Path -- //

  localparam [WSB:0] WR_SHIFTS = DDR_CWL - PHY_WR_LATENCY - 2;

  wire store_w, wr_rdy_w;

  assign store_w = ctl_cmd_i == CMD_WRIT && ready;

  always @(posedge clock) begin
    if (reset) begin
      wr_delay <= {WDLYS{1'b0}};
      wr_strob <= 1'b0;
      wr_ready <= 1'b0;
    end else begin
      if (store_w) begin
        wr_delay <= {WDLYS{1'b1}};
      end else begin
        wr_delay <= {1'b0, wr_delay[WSB:1]};
      end

      wr_strob <= wr_rdy_w;
      wr_ready <= wr_strob;
    end
  end

  shift_register #(
      .WIDTH(1),
      .DEPTH(16)
  ) wr_srl_inst (
      .clock (clock),
      .wren_i(1'b1),
      .addr_i(WR_SHIFTS),
      .data_i(wr_delay[0]),
      .data_o(wr_rdy_w)
  );


  // -- Read Data-Path -- //

  localparam [WSB:0] RD_SHIFTS = DDR_CL - PHY_RD_LATENCY - 2;

  wire fetch_w, rd_rdy_w;

  assign fetch_w = ctl_cmd_i == CMD_READ && ready;

  always @(posedge clock) begin
    if (reset) begin
      rd_delay <= {WDLYS{1'b0}};
      rd_ready <= 1'b0;
    end else begin
      if (fetch_w) begin
        rd_delay <= {WDLYS{1'b1}};
      end else begin
        rd_delay <= {1'b0, rd_delay[WSB:1]};
      end
      rd_ready <= rd_rdy_w;
    end
  end

  shift_register #(
      .WIDTH(1),
      .DEPTH(16)
  ) rd_srl_inst (
      .clock (clock),
      .wren_i(1'b1),
      .addr_i(RD_SHIFTS),
      .data_i(rd_delay[0]),
      .data_o(rd_rdy_w)
  );


  // -- Simulation Only -- //

`ifdef __icarus
  reg [79:0] dbg_state;

  always @* begin
    case (state)
      ST_IDLE: dbg_state = ddr_cke_i & ~ddr_cs_ni & ~reset ? "IDLE" : "INIT";
      ST_ACTV: dbg_state = "ACTIVATE";
      ST_READ: dbg_state = "READ";
      ST_WRIT: dbg_state = "WRITE";
      ST_PREC: dbg_state = "PRECHARGE";
      ST_PREA: dbg_state = "PRE-ALL";
      ST_REFR: dbg_state = "REFRESH";
      ST_ZQCL: dbg_state = "ZQCL";
      ST_MODE: dbg_state = "MODE REG";
      default: dbg_state = "UNKNOWN";
    endcase
  end

  reg [39:0] dbg_cmd;

  always @* begin
    case (cmd_q)
      CMD_MODE: dbg_cmd = "MRS";
      CMD_REFR: dbg_cmd = "REF";
      CMD_PREC: dbg_cmd = adr_q[10] ? "PREA" : "PRE";
      CMD_ACTV: dbg_cmd = "ACT";
      CMD_WRIT: dbg_cmd = adr_q[10] ? "WR-A" : "WR";
      CMD_READ: dbg_cmd = adr_q[10] ? "RD-A" : "RD";
      CMD_ZQCL: dbg_cmd = "ZQCL";
      CMD_NOOP: dbg_cmd = "---";
      default:  dbg_cmd = "XXX";
    endcase
  end
`endif


endmodule  // ddr3_ddl
