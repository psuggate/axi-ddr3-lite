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
    cfg_ba_i,
    cfg_adr_i,

    ddl_req_o,
    ddl_seq_o,
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
  input cfg_run_i;
  input cfg_req_i;
  output cfg_rdy_o;
  input [2:0] cfg_cmd_i;
  input [2:0] cfg_ba_i;
  input [RSB:0] cfg_adr_i;

  // DDR Data-Layer control signals
  // Note: all state-transitions are gated by the 'ddl_rdy_i' signal
  output ddl_req_o;
  output ddl_seq_o;
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
  localparam [3:0] ST_IDLE = 4'b0001;
  localparam [3:0] ST_READ = 4'b0010;
  localparam [3:0] ST_WRIT = 4'b0011;
  localparam [3:0] ST_PREC = 4'b0100;
  localparam [3:0] ST_PREA = 4'b0110;
  localparam [3:0] ST_ACTV = 4'b0111;
  localparam [3:0] ST_REFR = 4'b1000;


  reg wrack, rdack, byack;
  wire byp_req, mem_req, refresh;

  reg req_q, req_x, req_s, seq_q, seq_x, seq_s;
  reg [2:0] cmd_q, cmd_x, cmd_s, ba_q, ba_x, ba_s;
  reg [RSB:0] adr_q, adr_x, col_s;

  wire auto_w;
  wire [RSB:0] row_w, col_w;
  wire [2:0] bank_w;

  reg store, fetch;
  reg [3:0] state, snext;


  assign cfg_rdy_o = ddl_rdy_i;

  assign mem_wrack_o = wrack;
  assign mem_rdack_o = rdack;
  assign byp_rdack_o = byack;

  assign ddl_req_o = req_q;
  assign ddl_seq_o = seq_q;
  assign ddl_cmd_o = cmd_q;
  assign ddl_ba_o = ba_q;
  assign ddl_adr_o = adr_q;


  // -- Address Logic -- //

  wire [CSB:0] col_l, cfg_col_w;
  wire [1:0] asel;

  assign cfg_col_w = {DDR_COL_BITS{1'bx}};

  assign asel = !cfg_run_i ? 2'b11
              : byp_rdreq_i & BYPASS_ENABLE ? 2'b00
              : mem_rdreq_i ? 2'b01
              : mem_wrreq_i ? 2'b10
              : BYPASS_ENABLE ? 2'b00
              : 2'b01 ;

  assign {row_w, bank_w, col_l} = asel == 2'b11 ? {cfg_adr_i, cfg_ba_i, cfg_col_w}
                                : asel == 2'b10 ? mem_wradr_i
                                : asel == 2'b01 ? mem_rdadr_i
                                : byp_rdadr_i ;
  assign {col_w[RSB:11], col_w[9:0]} = {{(DDR_ROW_BITS - DDR_COL_BITS - 1) {1'b0}}, col_l};
  assign col_w[10] = auto_w;

  assign auto_w = byp_rdreq_i & BYPASS_ENABLE ? byp_rdlst_i
                : mem_rdreq_i ? mem_rdlst_i
                : mem_wrreq_i & mem_wrlst_i;


  // -- Main State Machine -- //

  always @(posedge clock) begin
    if (!cfg_run_i) begin
      // Forward the initialisation and configuration commands on to the DFI,
      // until the configuration module asserts 'cfg_run_i'.
      state <= ST_IDLE;
      store <= 1'b0;
      fetch <= 1'b0;
      req_q <= cfg_req_i;
      seq_q <= 1'b0;
      cmd_q <= cfg_cmd_i;
      ba_q  <= bank_w;
      adr_q <= row_w;
    end else begin
      case (state)
        ST_IDLE: begin
          // Wait for read-/write- requests -- refresh and precharging, as required
          if (refresh && !byp_req) begin
            // Note: we do not 'IDLE' with 'ACTIVE' banks
            state <= ST_REFR;
            store <= 1'b0;
            fetch <= 1'b0;
            req_q <= 1'b1;
            cmd_q <= CMD_REFR;
          end else if (byp_req || mem_rdreq_i && !mem_wrreq_i) begin
            state <= ST_ACTV;
            store <= 1'b0;
            fetch <= 1'b1;
            req_q <= 1'b1;  // 1st command, RAS#
            cmd_q <= CMD_ACTV;
          end else if (mem_wrreq_i) begin
            state <= ST_ACTV;
            store <= 1'b1;
            fetch <= 1'b0;
            req_q <= 1'b1;  // 1st command, RAS#
            cmd_q <= CMD_ACTV;
          end else begin
            state <= ST_IDLE;
            store <= 1'b0;
            fetch <= 1'b0;
            req_q <= 1'b0;
            cmd_q <= CMD_NOOP;
          end
          seq_q <= 1'b0;
          ba_q  <= bank_w;
          adr_q <= row_w;
        end

        ST_ACTV: begin
          // Row-activation command issued
          if (ddl_rdy_i) begin
            state <= snext;
            req_q <= req_x;
            seq_q <= seq_x;
            cmd_q <= cmd_x;
            ba_q  <= ba_x;
            adr_q <= adr_x;
          end
        end

        ST_READ: begin
          if (ddl_rdy_i) begin
            state <= snext;
            req_q <= req_x;
            seq_q <= seq_x;
            cmd_q <= cmd_x;
            ba_q  <= ba_x;
            adr_q <= adr_x;
          end
        end

        ST_WRIT: begin
          if (ddl_rdy_i) begin
            state <= snext;
            req_q <= req_x;
            seq_q <= seq_x;
            cmd_q <= cmd_x;
            ba_q  <= ba_x;
            adr_q <= adr_x;
          end
        end

        ST_PREC: begin
          // PRECHARGE-delay, following an auto-PRECHARGE ??
          // todo: this pattern shows up alot, make moar-betterer ?!
          if (ddl_rdy_i) begin
            state <= snext;
            req_q <= req_x;
            cmd_q <= cmd_x;
          end
          seq_q <= 1'bx;
        end

        ST_PREA: begin
          // PRECHARGE ALL banks (usually preceding a self-REFRESH)
          if (ddl_rdy_i) begin
            state <= snext;
            req_q <= req_x;
            cmd_q <= cmd_x;
            ba_q  <= ba_x;
            adr_q <= adr_x;
          end
          seq_q <= 1'bx;
        end

        ST_REFR: begin
          // Wait for all outstanding REFRESH operations to complete
          // Note: 'ddl_ref_i' stays asserted until REFRESH is about to finish
          if (!ddl_ref_i || ddl_rdy_i) begin
            state <= snext;
            req_q <= req_x;
            cmd_q <= cmd_x;
            ba_q  <= ba_x;
            adr_q <= adr_x;
          end else begin
            state <= state;
          end
          seq_q <= 1'bx;
        end

        default: begin
          $error("Oh noes");
          state <= ST_IDLE;
          seq_q <= 1'bx;
          // #100 $fatal;
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

        ST_WRIT: begin
          if (ddl_rdy_i && req_x) begin
            wrack <= 1'b1;
          end else begin
            wrack <= 1'b0;
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


  // -- Next-Command Logic -- //

  assign byp_req = byp_rdreq_i & BYPASS_ENABLE;
  assign mem_req = mem_wrreq_i | mem_rdreq_i;
  assign refresh = ddl_ref_i;

  always @(posedge clock) begin
    if (!cfg_run_i) begin
      snext <= ST_IDLE;
      seq_x <= 1'b0;
      req_x <= 1'b0;
      cmd_x <= CMD_NOOP;
      ba_x  <= 'bx;
      adr_x <= 'bx;
    end else begin
      case (state)
        ST_IDLE: begin
          if (!byp_req && (refresh || !mem_req)) begin
            snext <= ST_IDLE;
            seq_x <= 1'b0;
            req_x <= 1'b0;
            cmd_x <= 'bx;
            ba_x  <= 'bx;
            adr_x <= 'bx;
          end else begin
            req_x <= 1'b1;
            ba_x  <= bank_w;
            adr_x <= col_w;

            if (byp_req || !mem_wrreq_i) begin
              snext <= ST_READ;
              seq_x <= byp_req ? ~byp_rdlst_i : ~mem_rdlst_i;
              cmd_x <= CMD_READ;
            end else begin
              snext <= ST_WRIT;
              seq_x <= ~mem_wrlst_i;
              cmd_x <= CMD_WRIT;
            end
          end
        end

        ST_ACTV, ST_READ, ST_WRIT: begin
          if (ddl_rdy_i) begin
            snext <= req_s ? (fetch ? ST_READ : store ? ST_WRIT : 'bx) : ST_IDLE;
            seq_x <= seq_s;
            req_x <= req_s;
            cmd_x <= cmd_s;
            ba_x  <= ba_s;
            adr_x <= col_s;
          end
        end

        default: begin
          if (ddl_rdy_i) begin
            snext <= ST_IDLE;
            seq_x <= 1'b0;
            req_x <= 1'b0;
            cmd_x <= 'bx;
            ba_x  <= 'bx;
            adr_x <= 'bx;
          end
        end
      endcase
    end
  end

// Determines if any requests (RD, WR, BYP) allow for additional sequences of
// commands to be issued, for any of the open banks/rows.
  // todo:
  always @(posedge clock) begin
    if (!cfg_run_i) begin
      seq_s <= 1'b0;
      req_s <= 1'b0;
      cmd_s <= 'bx;
      ba_s  <= 'bx;
      col_s <= 'bx;
    end else begin
      case (state)
        ST_IDLE: begin
          if (byp_req) begin
            seq_s <= ~byp_rdlst_i;
          end else if (refresh) begin
            seq_s <= 1'b0;
          end else if (mem_wrreq_i) begin
            seq_s <= ~mem_wrlst_i;
          end else if (mem_rdreq_i) begin
            seq_s <= ~mem_rdlst_i;
          end else begin
            seq_s <= 1'b0;
          end
          req_s <= 1'b0;
          cmd_s <= 'bx;
          ba_s  <= 'bx;
          col_s <= 'bx;
        end

        ST_ACTV: begin
          if (seq_x && store && mem_wrreq_i) begin
            seq_s <= ~mem_wrlst_i;
            req_s <= 1'b1;
            cmd_s <= cmd_x;
            ba_s  <= ba_x;
            col_s <= col_w;
          end else if (seq_x && fetch && mem_rdreq_i) begin
            seq_s <= ~mem_rdlst_i;
            req_s <= 1'b1;
            cmd_s <= cmd_x;
            ba_s  <= ba_x;
            col_s <= col_w;
          end else begin
            seq_s <= 1'b0;
            req_s <= 1'b0;
            cmd_s <= 'bx;
            ba_s  <= 'bx;
            col_s <= 'bx;
          end
        end

        ST_WRIT: begin
          if (seq_x && store && mem_wrreq_i) begin
            seq_s <= ~mem_wrlst_i;
            req_s <= 1'b1;
            cmd_s <= cmd_x;
            ba_s  <= ba_x;
            col_s <= col_w;
          end else begin
            seq_s <= 1'b0;
            req_s <= 1'b0;
            cmd_s <= 'bx;
            ba_s  <= 'bx;
            col_s <= 'bx;
          end
        end

        default: begin
          seq_s <= 1'b0;
          req_s <= 1'b0;
          cmd_s <= 'bx;
          ba_s  <= 'bx;
          col_s <= 'bx;
        end
      endcase
    end
  end


  // -- Simulation Only -- //

`ifdef __icarus
  reg [39:0] dbg_state, dbg_snext;

  always @* begin
    case (state)
      ST_IDLE: dbg_state = cfg_run_i ? "IDLE" : "INIT";
      ST_READ: dbg_state = adr_q[10] ? "RD-A" : "RD";
      ST_WRIT: dbg_state = adr_q[10] ? "WR-A" : "WR";
      ST_PREC: dbg_state = "PRE";
      ST_PREA: dbg_state = "PREA";
      ST_ACTV: dbg_state = "ACT";
      ST_REFR: dbg_state = "REF";
      default: dbg_state = "XXX";
    endcase
    case (snext)
      ST_IDLE: dbg_snext = cfg_run_i ? "IDLE" : "INIT";
      ST_READ: dbg_snext = adr_q[10] ? "RD-A" : "RD";
      ST_WRIT: dbg_snext = adr_q[10] ? "WR-A" : "WR";
      ST_PREC: dbg_snext = "PRE";
      ST_PREA: dbg_snext = "PREA";
      ST_ACTV: dbg_snext = "ACT";
      ST_REFR: dbg_snext = "REF";
      default: dbg_snext = "XXX";
    endcase
  end

  wire [ 2:0] dbg_cmd_w = ddl_rdy_i ? cmd_q : CMD_NOOP;
  reg  [39:0] dbg_cmd;

  always @* begin
    case (dbg_cmd_w)
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


endmodule  // ddr3_fsm
