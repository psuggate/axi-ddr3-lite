`timescale 1ns / 100ps
/**
 * Issues DDR3 commands based on the read-request and write-request FIFO inputs.
 *
 * Specifics of DDR3 initialisation and timings are handled one level down, by
 * the "DFI" module. This module schedules row-activations, bank-precharging,
 * read-requests, and data-writes.
 * 
 * Note:
 *  - if enabled, "fast-path" read-requests can use the bypass ('byp_*') ports
 *    for low-latency, high-priority reads; e.g., to service fetch-requests from
 *    a processor (i.e., one of its caches);
 */
module ddr3_fsm (
    clock,
    reset,

    mem_wrreq_i,  // Write port
    mem_wrlst_i,
    mem_wrtid_i,
    mem_wradr_i,
    mem_wrack_o,
    mem_wrerr_o,

    mem_rdreq_i,  // Read port
    mem_rdlst_i,
    mem_rdtid_i,
    mem_rdadr_i,
    mem_rdack_o,
    mem_rderr_o,

    byp_rdreq_i,  // Read bypass-/fast- path port
    byp_rdlst_i,
    byp_rdtid_i,
    byp_rdadr_i,
    byp_rdack_o,
    byp_rderr_o,

    cfg_run_i,  // Configuration port
    cfg_req_i,
    cfg_rdy_o,
    cfg_cmd_i,
    cfg_ref_i,
    cfg_ba_i,
    cfg_adr_i,

    ddl_req_o,
    ddl_ref_i,
    ddl_rdy_i,
    ddl_cmd_o,
    ddl_tid_o,
    ddl_ba_o,
    ddl_adr_o
);

  // DDR3 SRAM Timings
  parameter DDR_FREQ_MHZ = 100;
  `include "ddr3_settings.vh"

  // Enables the (read-) bypass port
  // todo:
  parameter BYPASS_ENABLE = 1'b0;

  // If enabled, the {wrlst, rdlst} are used to decide when to ACTIVATE and
  // PRECHARGE rows
  // todo:
  parameter WRLAST_ENABLE = 1'b0;
  parameter RDLAST_ENABLE = 1'b0;
  parameter BYLAST_ENABLE = 1'b0;

  // Allows reads to be moved ahead of writes, and writes moved ahead of reads
  // (for same-bank accesses), in order to reduce turn-around costs
  // todo:
  //  - design & implement;
  //  - is this a good idea; i.e., is it worth the resources & complexity ??
  parameter OUT_OF_ORDER = 0;

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
  input mem_wrlst_i;  // If asserted, then LAST of burst
  output mem_wrack_o;
  output mem_wrerr_o;
  input [ISB:0] mem_wrtid_i;
  input [ASB:0] mem_wradr_i;

  // Read-request port
  input mem_rdreq_i;
  input mem_rdlst_i;  // If asserted, then LAST of burst
  output mem_rdack_o;
  output mem_rderr_o;
  input [ISB:0] mem_rdtid_i;
  input [ASB:0] mem_rdadr_i;

  // Bypass (fast-read) port
  input byp_rdreq_i;
  input byp_rdlst_i;  // If asserted, then LAST of burst
  output byp_rdack_o;
  output byp_rderr_o;
  input [ISB:0] byp_rdtid_i;
  input [ASB:0] byp_rdadr_i;

  // Configuration port
  input cfg_req_i;
  input cfg_run_i;
  output cfg_rdy_o;
  input [2:0] cfg_cmd_i;
  input cfg_ref_i;
  input [2:0] cfg_ba_i;
  input [RSB:0] cfg_adr_i;

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


  wire same_page;
  wire precharge_required;


  // DDR3 controller states
  localparam [3:0] ST_INIT = 4'b0000;
  localparam [3:0] ST_IDLE = 4'b0001;
  localparam [3:0] ST_READ = 4'b0010;
  localparam [3:0] ST_WRIT = 4'b0011;
  localparam [3:0] ST_PREC = 4'b0100;
  localparam [3:0] ST_PREA = 4'b0110;
  localparam [3:0] ST_ACTV = 4'b0111;
  localparam [3:0] ST_REFR = 4'b1000;


reg wrack, rdack, byack, req_q, req_x;

  reg [2:0] prev_wr_bank, prev_rd_bank;
  reg [2:0] cmd_q, cmd_x, ba_q, ba_x;
reg [RSB:0] adr_q, adr_x;

  reg [7:0] bank_actv;  // '1' if activated
  reg [RSB:0] actv_rows[0:7];  // row-address for each active bank

wire [RSB:0] row_w, col_w;
wire [2:0] bank_w;
  wire banks_active, bank_switch;


assign mem_wrack_o = wrack;
assign mem_rdack_o = rdack;
assign byp_rdack_o = byack;

assign ddl_req_o = req_q;
assign ddl_cmd_o = cmd_q;
assign ddl_ba_o  = ba_q;
assign ddl_adr_o = adr_q;


assign bank_w = mem_wradr_i[12:10];
assign row_w = mem_wradr_i[ASB:13];
assign col_w = {2'b00, autop, mem_wradr_i[9:0]};


  // -- Main State Machine -- //

  reg [3:0] state, snext;
  reg autop;

  assign banks_active = |bank_actv;

  always @(posedge clock) begin
    if (reset) begin
      state <= ST_INIT;
      snext <= 'bx;
      req_q <= 1'b0;
      cmd_q <= CMD_NOOP;
      req_x <= 1'b0;
      cmd_x <= CMD_NOOP;
      ba_q  <= 'bx;
      ba_x  <= 'bx;
      adr_q <= 'bx;
      adr_x <= 'bx;
    end else begin
      case (state)
        ST_INIT: begin
          // Sets the (post-RESET#) operating-mode for the device
          // Power-up initialisation is handled by the DFI state-machine, and
          // then the mode-setting state-machine (below)
          if (cfg_run_i) begin
            state <= ST_IDLE;
          end else begin
            state <= state;
          end
          snext <= 'bx;
        end

        ST_IDLE: begin
          // Wait for read-/write- requests -- refresh and precharging, as required
          if (ddl_ref_i) begin
            if (banks_active) begin
              state <= ST_PREA;
              snext <= ST_REFR;
              req_q <= 1'b1;
              cmd_q <= CMD_PREC;
              req_x <= 1'b1;
              cmd_x <= CMD_REFR;
            end else begin
              state <= ST_REFR;
              snext <= ST_IDLE;
              req_q <= 1'b1;
              cmd_q <= CMD_REFR;
              req_x <= 1'b0;
              cmd_x <= CMD_NOOP;
            end
          end else if (bank_switch) begin
            // todo: don't PRE from IDLE ??
            state <= ST_PREC;
            snext <= ST_IDLE;
            req_q <= 1'b1;
            cmd_q <= CMD_PREC;
            req_x <= 1'b0;
            cmd_x <= CMD_NOOP;
          end else if (byp_rdreq_i || mem_rdreq_i && !mem_wrreq_i) begin
            state <= ST_ACTV;
            snext <= ST_READ;
            req_q <= 1'b1; // 1st command, RAS#
            cmd_q <= CMD_ACTV;
            ba_q  <= bank_w;
            adr_q <= row_w;
            req_x <= 1'b1; // 2nd command, CAS#
            cmd_x <= CMD_READ;
            ba_x  <= bank_w;
            adr_x <= col_w;
          end else if (mem_wrreq_i) begin
            state <= ST_ACTV;
            snext <= ST_WRIT;
            req_q <= 1'b1; // 1st command, RAS#
            cmd_q <= CMD_ACTV;
            ba_q  <= bank_w;
            adr_q <= row_w;
            req_x <= 1'b1; // 2nd command, CAS# + WE#
            cmd_x <= CMD_WRIT;
            ba_x  <= bank_w;
            adr_x <= col_w;
          end else begin
            state <= ST_IDLE;
            snext <= 'bx;
            req_q <= 1'b0;
            cmd_q <= CMD_NOOP;
          end
        end

        ST_ACTV: begin
          // Row-activation command issued
          if (ddl_rdy_i) begin
            state <= snext;
            req_q <= req_x;
            cmd_q <= cmd_x;
            ba_q  <= ba_x;
            adr_q <= adr_x;
            cmd_x <= CMD_NOOP; // todo: back-to-back reads/writes
            req_x <= 1'b0; // todo: back-to-back reads/writes

            if (autop) begin
              // todo: there are multiple banks, so this is not always a good
              //   choice?
              snext <= ST_IDLE;
            end
          end
        end

        ST_READ: begin
          if (ddl_rdy_i) begin
            state <= snext;
            snext <= ST_IDLE;  // todo
            req_q <= req_x;
            cmd_q <= cmd_x;
            ba_q  <= ba_x;
            adr_q <= adr_x;
            req_x <= 1'b0;
            cmd_x <= CMD_NOOP;
          end
        end

        ST_WRIT: begin
          if (ddl_rdy_i) begin
            state <= snext;
            snext <= ST_IDLE;  // todo
            req_q <= req_x;
            cmd_q <= cmd_x;
            ba_q  <= ba_x;
            adr_q <= adr_x;
            req_x <= 1'b0;
            cmd_x <= CMD_NOOP;
          end
        end

        ST_PREC: begin
          // PRECHARGE-delay, following an auto-PRECHARGE ??
          // todo: this pattern shows up alot, make moar-betterer ?!
          if (!ddl_rdy_i) begin
            state <= state;
            snext <= snext;
          end else begin
            state <= snext;
            snext <= ST_IDLE; // todo
            req_q <= req_x;
            cmd_q <= cmd_x;
            req_x <= 1'b0;
            cmd_x <= CMD_NOOP;
          end
        end

        ST_PREA: begin
          // PRECHARGE ALL banks (usually preceding a self-REFRESH)
          if (!ddl_rdy_i) begin
            state <= state;
            snext <= snext;
          end else begin
            state <= snext;
            snext <= ST_IDLE; // todo
            req_q <= req_x;
            cmd_q <= cmd_x;
            ba_q  <= ba_x;
            adr_q <= adr_x;
            req_x <= 1'b0;
            cmd_x <= CMD_NOOP;
          end
        end

        ST_REFR: begin
          // Wait for all outstanding REFRESH operations to complete
          // Note: 'ddl_ref_i' stays asserted until REFRESH is about to finish
          if (!ddl_ref_i && ddl_rdy_i) begin
            state <= snext;
            snext <= ST_IDLE; // todo
            req_q <= req_x;
            cmd_q <= cmd_x;
            ba_q  <= ba_x;
            adr_q <= adr_x;
            req_x <= 1'b0;
            cmd_x <= CMD_NOOP;
          end else begin
            state <= state;
            snext <= snext;
          end
        end

        default: begin
          $error("Oh noes");
          state <= ST_INIT;
          snext <= 'bx;
          #100 $fatal;
        end
      endcase
    end
  end


  // -- AUTO-PRECHARGE State Machine -- //

  always @(posedge clock) begin
    if (reset) begin
      autop <= 1'b0;
    end else begin
      case (state)
        default: begin
          autop <= 1'b0;
        end

        ST_IDLE: begin
          // If a memory-command will be issued, is it the last in a sequence,
          // or do subsequent memory operations use the same '{row, bank}'?
          if (!ddl_ref_i && ddl_rdy_i) begin
            if (byp_rdreq_i) begin
              autop <= ~byp_rdlst_i;
            end else if (mem_rdreq_i) begin
              autop <= ~mem_rdlst_i;
            end else if (mem_wrreq_i) begin
              autop <= ~mem_wrlst_i;
            end else begin
              // todo: configuration modes ??
              autop <= 1'b0;
            end
          end else begin
            autop <= 1'b0;
          end
        end

        ST_ACTV: begin
          // Row-activation command issued
          if (ddl_rdy_i) begin
            // todo: if the next command is part of a burst, then keep 'autop'
            //   asserted (if it already is)
          end
        end

        ST_READ: begin
          if (ddl_rdy_i) begin
            // todo: if the next command is part of a burst, then keep 'autop'
            //   asserted (if it already is)
          end
        end

        ST_WRIT: begin
          if (ddl_rdy_i) begin
            // todo: if the next command is part of a burst, then keep 'autop'
            //   asserted (if it already is)
          end
        end
      endcase
    end
  end


// -- Acknowledge Signals -- //

  always @(posedge clock) begin
    if (reset) begin
      wrack <= 1'b0;
      rdack <= 1'b0;
      byack <= 1'b0;
    end else begin
      case (state)
        ST_IDLE: begin
          if (byp_rdreq_i) begin
            wrack <= 1'b0;
            rdack <= 1'b0;
            byack <= 1'b1;
          end else if (mem_rdreq_i) begin
            wrack <= 1'b0;
            rdack <= 1'b1;
            byack <= 1'b0;
          end else if (mem_wrreq_i) begin
            wrack <= 1'b1;
            rdack <= 1'b0;
            byack <= 1'b0;
          end else begin
            wrack <= 1'b0;
            rdack <= 1'b0;
            byack <= 1'b0;
          end
        end

          default: begin
            wrack <= 1'b0;
            rdack <= 1'b0;
            byack <= 1'b0;
          end
            
      endcase
    end
  end


  // -- Simulation Only -- //

`ifdef __icarus
  reg [79:0] dbg_state;

  always @* begin
    case (state)
      ST_INIT: dbg_state = "INIT";
      ST_IDLE: dbg_state = "IDLE";
      ST_READ: dbg_state = "RD";
      ST_WRIT: dbg_state = "WR";
      ST_PREC: dbg_state = "PRE";
      ST_PREA: dbg_state = "PREA";
      ST_ACTV: dbg_state = "ACT";
      ST_REFR: dbg_state = "REF";
      default: dbg_state = "UNKNOWN";
    endcase
  end
`endif


endmodule  // ddr3_fsm
