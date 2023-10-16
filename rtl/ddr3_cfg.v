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
  reg [2:0] refresh_pending;

  reg req_q, run_q;
  reg rst_nq, cke_q, cs_nq;

  reg [3:0] cmd_prev_q, cmd_curr_q;
  wire [3:0] cmd_next_w;


  assign ctl_run_o  = run_q;
  assign ctl_req_o  = req_q;

  assign dfi_rst_no = 1'bx;  // toods ...
  assign dfi_cke_o  = cke_q;
  assign dfi_cs_no  = cs_nq;
  assign dfi_odt_o  = 1'b0;


  // -- Chip Enable -- //

  always @(posedge clock) begin
    if (reset) begin
      cke_q <= 1'b0;
    end else begin
      if (dfi_rst_no == 1'b0) begin
        cke_q <= 1'b1;  // toods
      end
    end
  end


  // -- Initialisation and Refresh Counter -- //

  localparam CYCLES_MODE_SET = 4 * (DDR_CMRD + DDR_CMOD) + 2;
  localparam CYCLES_UNSTABLE = (600000 + TCK - 1) / TCK;
  localparam CYCLES_STABLE = COUNTER_INIT - UNSTABLE;

  localparam COUNTER_INIT = CYCLES_UNSTABLE + CMODES + DDR_CZQINIT + 2;
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
        // After RESET, reuse the counter for refresh intervals
        count <= COUNTER_REFI;
      end
    end
  end


  // -- Initialisation State Machine -- //

  // todo:
  //  - CKE: 0 -> 1 in 500 us
  //  - mode-register read & write sequencing

  localparam [3:0] ST_INIT = 4'b0000;  // power to stabilise
  localparam [3:0] ST_RSTN = 4'b0000;  // internal SDRAM reset
  localparam [3:0] ST_CKE1 = 4'b0001;  // start clock
  localparam [3:0] ST_MODE = 4'b0010;  // mode setting
  localparam [3:0] ST_ZQCL = 4'b0100;  // calibration
  localparam [3:0] ST_PREA = 4'b1101;  // PRECHARGE all
  localparam [3:0] ST_DONE = 4'b1000;  // hand over to mem. ctrl.

  reg [3:0] state;

  always @(posedge clock) begin
    if (reset) begin
      state  <= ST_INIT;
      req_q  <= 1'b0;
      run_q  <= 1'b0;
      rst_nq <= 1'b0;
      cke_q  <= 1'b0;
      cs_nq  <= 1'b1;
    end else begin
      case (state)
        ST_INIT: begin
          req_q <= 1'b0;
          run_q <= 1'b0;
        end

        ST_RSTN: begin
          // Allow for power supply to stablise
          rst_nq <= 1'b0;

          if (count < STABLE) begin
            state <= ST_CKE1;
            cke_q <= 1'b1;
          end
        end

        ST_CKE1: begin
          // After > 5 ticks, deassert RESET#
        end

        ST_MODE: begin
        end

        ST_ZQCL: begin
          // Wait for the DDR3 device to calibrate the impedance of its data-
          // output drivers
          if (count == 0) begin
            state <= ST_PREA;
          end
        end

        ST_PREA: begin
          // Wait until timer has elapsed
          state <= ST_DONE;
        end

        ST_DONE: begin
          // Chill here until RESET# asserts ...
          state <= ST_DONE;
        end
      endcase
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
          if (state == ST_MODE) begin
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


endmodule  // ddr3_cfg
