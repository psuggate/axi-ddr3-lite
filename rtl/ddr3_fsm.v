`timescale 1ns / 100ps
/**
 * Issues DDR3 commands based on the read-request and write-request FIFO inputs.
 *
 * Specifics of DDR3 initialisation and timings are handled one level down, by
 * the "DFI" module. This module schedules row-activations, bank-precharging,
 * read-requests, and data-writes.
 */
module ddr3_fsm (  /*AUTOARG*/);

  // DDR3 SRAM Timings
  parameter DDR_FREQ_MHZ = 100;
  localparam TCK = 1000 / DDR_FREQ_MHZ;

  // Allows reads to be moved ahead of writes, and writes moved ahead of reads
  // (for same-bank accesses), in order to reduce turn-around costs
  parameter OUT_OF_ORDER = 0;

  // Wait for the DFI to enable us to issue the next command; OR, queue up as many
  // commands as possible?
  // todo: does this have throughput consequences, or just pointless ??
  parameter DFI_GATING = 1;  // todo: ??

  // Could be useful, if 'BL8' transfers are not (always) useful, for some use-
  // case, or memory configuration.
  parameter BURST_CHOP = 0;  // todo: ??

  // Data-path and address settings
  parameter WIDTH = 32;
  localparam MSB = WIDTH - 1;

  parameter MASKS = WIDTH / 8;
  localparam SSB = MASKS - 1;

  // Request ID's are represent the order that commands are accepted at the bus/
  // transaction layer, and if used, the memory controller will respect this
  // ordering, when reads and writes access overlapping areas of memory.
  parameter REQID = 4;
  localparam ISB = REQID - 1;

  // Defaults for 1Gb, 16x SDRAM
  parameter DDR_ROW_BITS = 13;
  localparam RSB = DDR_ROW_BITS - 1;
  parameter DDR_COL_BITS = 10;
  localparam CSB = DDR_COL_BITS - 1;

  // Default is '{row, bank, col} <= addr_i;' -- this affects how often banks will
  // need PRECHARGE commands. Enabling this alternate {bank, row, col} ordering
  // may help for some workloads; e.g., when all but the upper (burst) addresses
  // are not correlated in time?
  parameter BANK_ROW_COL = 0;

  // Note: all addresses for requests must be word- and burst- aligned. Therefore,
  //   for a x16 DDR3 device, each transfer is 16 bytes, so the lower 4-bits are
  //   not passed to this controller.
  // Note: a 1Gb, x16, DDR3 SDRAM has:
  //    - 13b row address bits;
  //    -  3b bank address bits;
  //    - 10b column address bits; and
  //    - 2kB page-size,
  //   and the lower 3b of the column address are ignored by this module. So a 23b
  //   address is required.
  // Todo: in order to support wrapping-bursts; e.g., for a CPU cache, will need
  //   the lower 3b ??
  parameter ADDRS = 23;
  localparam ASB = ADDRS - 1;


  input clock;  // Shared clock domain for the memory-controller
  input reset;  // Synchronous reset

  // Write-request port
  input mem_wrreq_i;
  output mem_wrack_o;
  output mem_wrerr_o;
  output [ISB:0] mem_wrtid_o;
  input [ASB:0] mem_wradr_i;

  // Read-request port
  input mem_rdreq_i;
  output mem_rdack_o;
  output mem_rderr_o;
  input [ISB:0] mem_rdtid_i;
  input [ASB:0] mem_rdadr_i;

  // DDR Data-Layer control signals
  // Note: all state-transitions are gated by the 'ddl_rdy_i' signal
  output ddl_req_o;
  input ddl_rdy_i;
  input ddl_ref_i;
  output [2:0] ddl_cmd_o;
  output [ISB:0] ddl_tid_o;
  output [2:0] ddl_ba_o;
  output [RSB:0] ddl_adr_o;


  // todo:
  //  - detect same bank+row, for subsequent commands
  //     + long-bursts that cross page boundaries ?
  //     + "coalesce" reads and/or writes to same pages ?
  //  - auto-precharge when required
  //  - command ordering (using transaction ID's)
  //  - scheduling for read- & write- ports:
  //     + command interleaving to hide ACTIVATE & PRECHARGE delays ?
  //     + "fairness"
  //  - track the active row for each bank
  //  - command-queuing ? I.e., given the read- and write- ports, can commands be
  //    determined, and pushed to a FIFO well-ahead of their actual dispatch ?
  //  - refresh issuing, as this can be flexible ?
  //  - which part of the address should map to the bank bits?


  // -- Constants -- //

  // Relative transaction costs, for scoring and scheduling
  parameter COST_RD_TO_RD = 2;
  parameter COST_WR_TO_WR = 2;
  parameter COST_RD_TO_WR = 3;
  parameter COST_WR_TO_RD = 7;

  // DDR3 Commands: {CS# = 1'b0, RAS#, CAS#, WE#}
  localparam DDR3_NOOP = 3'b111;
  localparam DDR3_ZQCL = 3'b110;
  localparam DDR3_READ = 3'b101;
  localparam DDR3_WRIT = 3'b100;
  localparam DDR3_ACTV = 3'b011;
  localparam DDR3_PREC = 3'b010;
  localparam DDR3_REFR = 3'b001;
  localparam DDR3_MODE = 3'b000;


  wire same_page;
  wire precharge_required;


  // DDR3 controller states
  localparam [3:0] ST_INIT = 4'b0000;
  localparam [3:0] ST_IDLE = 4'b0001;
  localparam [3:0] ST_READ = 4'b0010;
  localparam [3:0] ST_WRIT = 4'b0011;
  localparam [3:0] ST_PREC = 4'b0100;
  localparam [3:0] ST_PREA = 4'b0110;
  localparam [3:0] ST_REFR = 4'b1000;


  reg [2:0] prev_wr_bank, prev_rd_bank;

  reg [7:0] bank_actv;  // '1' if activated
  reg [RSB:0] actv_rows[0:7];  // row-address for each active bank


  // -- Main State Machine -- //

  reg [3:0] state;

  always @(posedge clock) begin
    if (reset) begin
      state <= ST_INIT;
    end else begin
      case (state)
        ST_INIT: begin
          // Sets the (post-RESET#) operating-mode for the device
          // Power-up initialisation is handled by the DFI state-machine, and
          // then the mode-setting state-machine (below)
          if (smode == MR_DONE) begin
            state <= ST_IDLE;
          end else begin
            state <= state;
          end
        end

        ST_IDLE: begin
          // Wait for read-/write- requests -- refresh and precharging, as required
          if (refresh_pending != 3'b000) begin
            state <= ST_PREA;
          end else if (bank_switch) begin
            state <= ST_PREC;
          end
          state <= ST_IDLE;
        end

        ST_READ: begin
          state <= ST_IDLE;
        end

        ST_WRIT: begin
          state <= ST_IDLE;
        end

        ST_PREC: begin
          // PRECHARGE-delay, following an auto-PRECHARGE ??
          // todo: this pattern shows up alot, make moar-betterer ?!
          if (!ddr_rdy_i) begin
            state <= state;
          end else if (cmd_next_q == DDR_READ) begin
            state <= ST_READ;
          end else if (cmd_next_q == DDR_WRIT) begin
            state <= ST_WRIT;
          end else begin
            state <= ST_IDLE;
          end
        end

        ST_PREA: begin
          // PRECHARGE ALL banks (usually preceding a self-REFRESH)
          if (!ddr_rdy_i) begin
            state <= state;
          end else if (refresh_pending != 3'b000) begin
            state <= ST_REFR;
          end else begin
            state <= ST_IDLE;
          end
        end

        ST_REFR: begin
          // Wait for all outstanding self-REFRESH operations to complete
          if (!ddr_rdy_i) begin
            state <= state;
          end else if (refresh_pending == 3'b000) begin
            state <= ST_IDLE;
          end else begin
            state <= state;
          end
        end

        default: begin
          $error("Oh noes");
          $fatal;
          state <= ST_INIT;
        end
      endcase
    end
  end


  // -- Command-Queuing -- //

  reg [2:0] ddr_cmd_q, ddr_bank_q;
  reg [RSB:0] ddr_row_q;

  generate
    if (DFI_GATING) begin : g_no_queuing

      // Slightly smaller core
      assign ddr_cmd_o = ddr_cmd_q;

    end else begin : g_cmd_queues

      wire cmd_queue, cmd_ready, ddr_valid;
      wire [DDR_ROW_BITS + 5:0] cmd_yummy;

      assign cmd_yummy = {ddr_cmd_q, ddr_bank_q, ddr_row_q};

      // Queues up to '1 << ABITS' commands, and these are consumed by the DFI in
      // accordance to the timings of the DDR3 PHY.
      sync_fifo #(
          .WIDTH (3 + 3 + DDR_ROW_BITS),
          .ABITS (4),
          .OUTREG(1)
      ) dfi_cmd_fifo_inst (
          .clock  (clock),
          .reset  (reset),
          .valid_i(cmd_queue),
          .ready_o(cmd_ready),
          .data_i (cmd_yummy),
          .valid_o(ddr_valid),
          .ready_i(ddr_rdy_i),
          .data_o (ddr_cmd_o)
      );

    end  // g_cmd_queues
  endgenerate


endmodule  // ddr3_fsm
