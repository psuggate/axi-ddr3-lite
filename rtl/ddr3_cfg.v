`timescale 1ns / 100ps
/**
 * DDR3 configuration is here to emphasize that it has been moved out of the
 * "critical path" of the memory controller.
 *
 * Notes:
 *  - handles device- and mode- specific timings;
 *
 * Copyright 2023, Patrick Suggate.
 *
 */
module ddr3_cfg (
    clock,
    reset,

    cfg_valid_i,  // toods ??
    cfg_ready_o,
    cfg_data_i,
    cfg_data_o,

    dfi_rst_no,  // Control these IOB's directly
    dfi_cke_o,
    dfi_cs_no,
    dfi_odt_o,

    ctl_req_o,  // Memory controller signals
    ctl_run_o,  // When initialisation has completed
    ctl_rdy_i,
    ctl_cmd_o,
    ctl_ref_o,
    ctl_ba_o,
    ctl_adr_o
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


  input clock;
  input reset;

  // On-the-fly setting updates (if enabled)
  input cfg_valid_i;
  output cfg_ready_o;
  input [31:0] cfg_data_i;
  output [31:0] cfg_data_o;

  // (Pseudo-) DDR3 PHY Interface (-ish)
  output dfi_rst_no;
  output dfi_cke_o;
  output dfi_cs_no;
  output dfi_odt_o;

  // From/to DDR3 Controller
  output ctl_req_o;
  output ctl_run_o;
  input ctl_rdy_i;
  output [2:0] ctl_cmd_o;
  output ctl_ref_o;
  output [2:0] ctl_ba_o;
  output [RSB:0] ctl_adr_o;


  // -- Constants -- //

  // REFRESH settings
  localparam CREFI = (DDR_TREFI - 1) / TCK;  // cycles(tREFI) - 1
  localparam RFC_BITS = $clog2(CREFI);
  localparam RFCSB = RFC_BITS - 1;
  localparam [RFCSB:0] RFC_ZERO = {RFC_BITS{1'b0}};


  reg [RFCSB:0] refresh_counter;
  reg [2:0] refresh_pending, cmd_q, ba_q;
  reg [RSB:0] adr_q;

  reg req_q, ref_q, run_q;
  reg rst_nq, cke_q, cs_nq;

  reg [3:0] cmd_prev_q, cmd_curr_q;
  wire [3:0] cmd_next_w;


  assign ctl_run_o  = run_q;
  assign ctl_req_o  = req_q;
  assign ctl_ref_o  = ref_q;
  assign ctl_cmd_o  = cmd_q;
  assign ctl_ba_o   = ba_q;
  assign ctl_adr_o  = adr_q;

  assign dfi_rst_no = rst_nq;  // toods ...
  assign dfi_cke_o  = cke_q;
  assign dfi_cs_no  = cs_nq;
  assign dfi_odt_o  = 1'b0;


  // -- Initialisation and Refresh Counter -- //
`ifdef __icarus
  // Faster start-up times for the impatient (simulator) ...
  localparam CYCLES_UNSTABLE = (10000 + TCK - 1) / TCK;
  localparam CYCLES_STARTUP = (50000 + TCK - 1) / TCK;
`else
  // Clock cycles required for power to stabilise (200 us)
  // Note: RESET# is asserted throughout this period
  localparam CYCLES_UNSTABLE = (200000 + TCK - 1) / TCK;

  // Clock cycles required for SDRAM to internal-RESET# (500 us)
  // Note: CKE has to be de-asserted (>= 5 cycles) prior to this phase
  localparam CYCLES_STARTUP = (500000 + TCK - 1) / TCK;
`endif

  // Clock cycles required to set all four mode registers
  localparam CYCLES_MODE_SET = 4 * (DDR_CMRD + DDR_CMOD) + 2;

  localparam COUNTER_INIT = CYCLES_UNSTABLE + CYCLES_STARTUP + CYCLES_MODE_SET + DDR_CZQINIT + 2;
  localparam COUNTER_REFI = CREFI;
  localparam COUNTER_ZERO = {COUNTER_BITS{1'b0}};

  localparam COUNTER_BITS = $clog2(COUNTER_INIT);
  localparam XSB = COUNTER_BITS - 1;

  reg  [XSB:0] count;
  wire [XSB:0] cnext;

  assign cnext = count - 1;

  always @(posedge clock) begin
    if (reset) begin
      count <= COUNTER_INIT;
    end else begin
      if (cnext != COUNTER_ZERO) begin
        count <= cnext;
      end else begin
        // After SDRAM startup, reuse the counter for refresh intervals
        count <= COUNTER_REFI;
      end
    end
  end

  wire refresh_issued = ctl_rdy_i && ref_q;

  always @(posedge clock) begin
    if (reset) begin
      refresh_pending <= 3'd0;
    end else if (run_q && cnext == COUNTER_ZERO) begin
      // REFRESH completed?
      if (refresh_issued) begin
        refresh_pending <= refresh_pending;
      end else begin
        refresh_pending <= refresh_pending + 1;
      end
    end else if (run_q) begin
      // REFRESH completed?
      if (refresh_issued && refresh_pending != 3'd0) begin
        refresh_pending <= refresh_pending - 1;
      end else begin
        refresh_pending <= refresh_pending;
      end
    end
  end

  always @(posedge clock) begin
    if (reset) begin
      ref_q <= 1'b0;
    end else begin
      ref_q <= |refresh_pending;
    end
  end


  // -- Initialisation State Machine -- //

  // todo:
  //  - CKE: 0 -> 1 in 500 us
  //  - mode-register read & write sequencing

  localparam [3:0] ST_RSTN = 4'b0000;  // internal SDRAM reset
  localparam [3:0] ST_INIT = 4'b0001;  // power to stabilise
  localparam [3:0] ST_CKE1 = 4'b0010;  // start clock
  localparam [3:0] ST_MRS2 = 4'b1010;  // set mode-register #2
  localparam [3:0] ST_MRS3 = 4'b1011;  // set mode-register #3
  localparam [3:0] ST_MRS1 = 4'b1001;  // set mode-register #1
  localparam [3:0] ST_MRS0 = 4'b1000;  // set mode-register #0
  localparam [3:0] ST_ZQCL = 4'b0011;  // calibration
  localparam [3:0] ST_PREA = 4'b0100;  // PRECHARGE all
  localparam [3:0] ST_DONE = 4'b0101;  // hand over to mem. ctrl.

  reg [3:0] state;

  localparam COUNTER_RSTN = COUNTER_INIT - CYCLES_UNSTABLE;
  localparam COUNTER_STRT = COUNTER_RSTN - CYCLES_STARTUP;

  always @(posedge clock) begin
    if (reset) begin
      state  <= ST_RSTN;
      req_q  <= 1'b0;
      run_q  <= 1'b0;
      rst_nq <= 1'b0;
      cke_q  <= 1'b0;
      cs_nq  <= 1'b1;
    end else begin
      case (state)
        ST_RSTN: begin
          // RESET# to allow for power supply to stablise
          req_q <= 1'b0;  // to mem. ctrl.
          run_q <= 1'b0;

          cke_q <= 1'b0;  // to SDRAM
          cs_nq <= 1'b1;

          if (count < COUNTER_RSTN) begin
            state  <= ST_INIT;
            rst_nq <= 1'b1;
          end else begin
            rst_nq <= 1'b0;  // to SDRAM
          end
        end

        ST_INIT: begin
          // SDRAM now begins its internal startup procedures
          rst_nq <= 1'b1;
          cs_nq  <= 1'b1;

          if (count < COUNTER_STRT) begin
            state <= ST_CKE1;
            cke_q <= 1'b1;
          end else begin
            cke_q <= 1'b0;
          end
        end

        ST_CKE1: begin
          // CKE now asserted, so start issuing MODE commands ...
          // Note: CKE -> MRS2 requires >= 5x tCK
          state <= ST_MRS2;
          cs_nq <= 1'b0;
          req_q <= 1'b1;
          cmd_q <= CMD_MODE;
          {ba_q, adr_q} <= {3'b010, MR2};
        end

        ST_MRS2: begin
          // Note: MRS2 -> MRS3 requires >= 12x tCK
          if (ctl_rdy_i) begin
            state <= ST_MRS3;
            req_q <= 1'b1;
            cmd_q <= CMD_MODE;
            {ba_q, adr_q} <= {3'b011, MR3};
          end
        end

        ST_MRS3: begin
          // Note: MRS3 -> MRS1 requires >= 12x tCK
          if (ctl_rdy_i) begin
            state <= ST_MRS1;
            req_q <= 1'b1;
            cmd_q <= CMD_MODE;
            {ba_q, adr_q} <= {3'b001, MR1};
          end
        end

        ST_MRS1: begin
          // Note: MRS1 -> MRS0 requires >= 12x tCK
          if (ctl_rdy_i) begin
            state <= ST_MRS0;
            req_q <= 1'b1;
            cmd_q <= CMD_MODE;
            {ba_q, adr_q} <= {3'b000, MR0};
          end
        end

        ST_MRS0: begin
          // Note: MRS0 -> ZQCL requires >= 12x tCK
          if (ctl_rdy_i) begin
            state <= ST_ZQCL;
            req_q <= 1'b1;
            cmd_q <= CMD_ZQCL;
            {ba_q, adr_q} <= {3'bx, 2'bx, 1'b1, 10'bx};
            // {ba_q, adr_q} <= 'bx;
          end
        end

        ST_ZQCL: begin
          // Wait for the DDR3 device to calibrate the impedance of its data-
          // output drivers
          if (ctl_rdy_i) begin
            state <= ST_PREA;
            req_q <= 1'b1;
            cmd_q <= CMD_PREC;
            {ba_q, adr_q} <= {3'bx, 2'bx, 1'b1, 10'bx};
          end
        end

        ST_PREA: begin
          // Wait until timer has elapsed
          if (ctl_rdy_i) begin
            state <= ST_DONE;
            req_q <= 1'b0;
            cmd_q <= CMD_NOOP;
            // {ba_q, adr_q} <= {3'h0, 2'h0, 1'b1, 10'h000};
          end
        end

        ST_DONE: begin
          // Chill here until RESET# asserts ...
          if (ctl_rdy_i) begin
            state <= ST_DONE;
            run_q <= 1'b1;
          end
        end

        default: begin
          $error("%10t: CFG: Unhandled CFG state: %1x", $time, state);
          state  <= ST_RSTN;
          rst_nq <= 1'b0;
          cke_q  <= 1'b0;
          cs_nq  <= 1'b1;
        end
      endcase
    end
  end


  // -- Refresh Counter -- //
  /*
  wire refresh_issued = ctl_cmd_o == CMD_REFR && ctl_rdy_i;

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
*/


  // -- Simulation Only -- //

`ifdef __icarus
  initial begin
    $display("COUNTER_BITS: %d", COUNTER_BITS);
    $display("COUNTER_INIT: %08x", COUNTER_INIT);
    $display("COUNTER_REFI: %08x", COUNTER_REFI);
  end

  reg [79:0] dbg_state;

  always @* begin
    case (state)
      ST_RSTN: dbg_state = "RESET";
      ST_INIT: dbg_state = "INIT";
      ST_CKE1: dbg_state = "CKE";
      ST_MRS2: dbg_state = "MRS #2";
      ST_MRS3: dbg_state = "MRS #3";
      ST_MRS1: dbg_state = "MRS #1";
      ST_MRS0: dbg_state = "MRS #0";
      ST_ZQCL: dbg_state = "ZQCL";
      ST_PREA: dbg_state = "PRECHARGE";
      ST_DONE: dbg_state = "DONE";
      default: dbg_state = "UNKNOWN";
    endcase
  end
`endif


endmodule  // ddr3_cfg
