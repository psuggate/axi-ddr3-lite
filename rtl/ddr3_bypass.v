`timescale 1ns / 100ps
/**
 * Read-only fast-path AXI4 interface to the memory controller. Bypasses the
 * ordinary address- and data- paths, for lower-latency reads.
 *
 * Note:
 *  - BOTH 'axi_arvalid_i' AND 'axi_rready_i' need to be asserted, to begin a
 *    fast-path read -- you will need to add an output FIFO if otherwise you
 *    cannot guarantee that the read-data will be accepted (without wait-
 *    states);
 *  - intended for low-latency, high-priority reads; e.g., to service fetch-
 *    requests from a processor (i.e., one of its caches);
 *  - handles only a subset of the SDRAM functionality;
 *  - throws an exception if the requesting core does not accept the returned
 *    read-data in time (so 'axirready_i' should be asserted, and held, until
 *    the transaction completes);
 *  - address alignment must be correct, and the AXI4 burst-size must be BL8;
 */
module ddr3_bypass (
    clock,
    reset,

    axi_arvalid_i,  // AXI4 fast-path, read-only port
    axi_arready_o,
    axi_araddr_i,
    axi_arid_i,
    axi_arlen_i,
    axi_arburst_i,

    axi_rready_i,
    axi_rvalid_o,
    axi_rlast_o,
    axi_rresp_o,
    axi_rid_o,
    axi_rdata_o,

    ddl_rvalid_i,  // DDL READ data-path
    ddl_rready_o,
    ddl_rlast_i,
    ddl_rdata_i,

    byp_run_i,  // Connects to the DDL
    byp_req_o,
    byp_seq_o,
    byp_ref_i,
    byp_rdy_i,
    byp_cmd_o,
    byp_tid_o,
    byp_ba_o,
    byp_adr_o,

    ctl_run_o,  // Intercepts these memory controller -> DDL signals
    ctl_req_i,
    ctl_seq_i,
    ctl_ref_o,
    ctl_rdy_o,
    ctl_cmd_i,
    ctl_ba_i,
    ctl_adr_i,

    ctl_rvalid_o,  // READ data from DDL -> memory controller data-path
    ctl_rready_i,
    ctl_rlast_o,
    ctl_rdata_o
);

  // todo:
  //  1. ~~disabled-mode works~~
  //  2. enabled-mode and no bypass-requests works
  //  3. works as read-only, single-BL8 AXI4 interface (ignores slow-path requests)
  //  4. multi-BL8 reads
  //  5. row & bank detection ??

  // DDR3 SRAM Timings
  parameter DDR_FREQ_MHZ = 100;
  `include "ddr3_settings.vh"

  parameter BYPASS_ENABLE = 1'b1;

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
  // todo: ...
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

  input axi_arvalid_i;  // AXI4 Fast-Read Address Port
  output axi_arready_o;
  input [ASB:0] axi_araddr_i;
  input [ISB:0] axi_arid_i;
  input [7:0] axi_arlen_i;
  input [1:0] axi_arburst_i;

  input axi_rready_i;  // AXI4 Fast-Read Data Port
  output axi_rvalid_o;
  output [MSB:0] axi_rdata_o;
  output [1:0] axi_rresp_o;
  output [ISB:0] axi_rid_o;
  output axi_rlast_o;

  input ddl_rvalid_i;
  output ddl_rready_o;
  input ddl_rlast_i;
  input [MSB:0] ddl_rdata_i;

  // DDR Data-Layer control signals
  // Note: all state-transitions are gated by the 'byp_rdy_i' signal
  input byp_run_i;
  output byp_req_o;
  output byp_seq_o;
  input byp_rdy_i;
  input byp_ref_i;
  output [2:0] byp_cmd_o;
  output [ISB:0] byp_tid_o;
  output [2:0] byp_ba_o;
  output [RSB:0] byp_adr_o;

  // From/to DDR3 Controller
  // Note: all state-transitions are gated by the 'ctl_rdy_o' signal
  output ctl_run_o;
  input ctl_req_i;
  input ctl_seq_i;  // Burst-sequence indicator
  output ctl_rdy_o;
  output ctl_ref_o;
  input [2:0] ctl_cmd_i;
  input [2:0] ctl_ba_i;
  input [RSB:0] ctl_adr_i;

  output ctl_rvalid_o;  // Read port, back to memory controller
  input ctl_rready_i;
  output ctl_rlast_o;
  output [MSB:0] ctl_rdata_o;


  reg arack, rdack;
  reg req_q, req_x, req_s, seq_q, seq_x, seq_s;
  reg [2:0] cmd_q, cmd_x, cmd_s, ba_q, ba_x, ba_s;
  reg [RSB:0] adr_q, adr_x, col_s;

  wire auto_w, bypass;
  wire [2:0] bank_w, cmd_w;
  wire [RSB:0] row_w, col_w, adr_w;

  reg [3:0] state, snext;


  generate
    if (BYPASS_ENABLE) begin : g_bypass

      // -- Use the Bypass Port -- //

      assign axi_arready_o = rdack;
      assign axi_rvalid_o = arack;

      assign byp_req_o = bypass | ctl_req_i;
      assign byp_seq_o = bypass ? seq_q : ctl_seq_i;  // todo
      assign byp_cmd_o = bypass ? cmd_w : ctl_cmd_i;
      assign byp_ba_o = bypass ? bank_w : ctl_ba_i;
      assign byp_adr_o = bypass ? row_w : ctl_adr_i;

      assign ctl_rdy_o = bypass ? 1'b0 : byp_rdy_i;  // todo
      assign ctl_ref_o = byp_ref_i;

    end else begin : g_normal

      // -- Bypass is Disabled -- //

      // Don't drive the AXI4 interface (in-case it is connected)
      assign axi_arready_o = 1'b0;

      assign axi_rvalid_o = 1'b0;
      assign axi_rlast_o = 1'b0;
      assign axi_rresp_o = 'bx;
      assign axi_rid_o = 'bx;
      assign axi_rdata_o = 'bx;

      // Forward all signals directly to/from the memory controller
      assign ctl_run_o = byp_run_i;
      assign ctl_rdy_o = byp_rdy_i;
      assign ctl_ref_o = byp_ref_i;

      assign byp_req_o = ctl_req_i;
      assign byp_seq_o = ctl_seq_i;
      assign byp_cmd_o = ctl_cmd_i;
      assign byp_ba_o = ctl_ba_i;
      assign byp_adr_o = ctl_adr_i;

      assign ctl_rvalid_o = ddl_rvalid_i;
      assign ctl_rlast_o = ddl_rlast_i;
      assign ctl_rdata_o = ddl_rdata_i;

      assign ddl_rready_o = ctl_rready_i;

    end
  endgenerate


  // -- Address Logic -- //

  wire [CSB:0] col_l;

  assign adr_w = req_q ? adr_q
               : axi_arvalid_i & axi_rready_i ? axi_araddr_i[ASB:ADDRS-DDR_ROW_BITS]
               : ctl_adr_i;

  assign auto_w = 1'b1;  // todo: gonna have to detect sequences ourselves ...

  assign {row_w, bank_w, col_l} = axi_araddr_i;
  assign {col_w[RSB:11], col_w[9:0]} = {{(DDR_ROW_BITS - DDR_COL_BITS - 1) {1'b0}}, col_l};
  assign col_w[10] = auto_w;


  // Capture the bypass-address for subsequent stages of the transaction
  always @(posedge clock) begin
    case (state)
      ST_IDLE: begin
        ba_q  <= bank_w;
        adr_q <= row_w;
      end

      ST_ACTV, ST_READ, ST_PREC: begin
        if (byp_rdy_i) begin
          ba_q  <= ba_x;
          adr_q <= adr_x;
        end
      end

      default: begin
        $error("%10t: Invalid state = 0x%1", $time, state);
        ba_q  <= 'bx;
        adr_q <= 'bx;
      end
    endcase
  end


  // -- PRECHARGE Detection -- //

  reg autop;
  reg active_q;
  reg [RSB:0] actv_row;
  reg [2:0] actv_ba;

  // If bypass request address requires a different ROW within the currently-
  // ACTIVE BANK, then we need to PRECHARGE, then re-ACTIVATE that same BANK
  // after the fast-path READ completes.

  always @(posedge clock) begin
    if (reset) begin
      autop <= 1'b0;
    end else begin
    end
  end

  // Track the currently-ACTIVE ROW & BANK.
  // Note: assumes that the memory controller only ever has one bank ACTIVE.
  // todo: is this robust enough ??

  always @(posedge clock) begin
    if (reset) begin
      active_q <= 1'b0;
      actv_ba  <= 'bx;
      actv_row <= 'bx;
    end else if (ctl_req_i && byp_rdy_i && ctl_cmd_i == CMD_ACTV) begin
      active_q <= 1'b1;
      actv_ba  <= ctl_ba_i;
      actv_row <= ctl_adr_i;
    end else if (ctl_req_i && byp_rdy_i && ctl_cmd_i == CMD_PREC) begin
      active_q <= 1'b0;
      actv_ba  <= 'bx;
      actv_row <= 'bx;
    end else begin
      active_q <= active_q;
      actv_ba  <= actv_ba;
      actv_row <= actv_row;
    end
  end


  // -- Burst-Sequence Detection -- //

  localparam [1:0] BURST_INCR = 2'b01;

  reg burst;

  // For AXI4 incrementing bursts, perform multiple, sequential (DDR3 SDRAM) BL8
  // reads.
  // todo: ...

  always @(posedge clock) begin
    if (reset) begin
      burst <= 1'b0;
    end else if (axi_arvalid_i && axi_rready_i) begin
      burst <= axi_arlen_i > 3 && axi_arburst_i == BURST_INCR;
    end else if (axi_rvalid_o && axi_rready_i && axi_rlast_o) begin
      burst <= 1'b0;
    end else begin
      burst <= burst;
    end
  end


  // -- Main State Machine -- //

  // todo:
  //  - detect same bank+row, for subsequent commands
  //     + long-bursts that cross page boundaries ?
  //     + "coalesce" reads and/or writes to same pages ?
  //  - auto-precharge when required
  //  - track the active row for each bank
  //  - overflow registers for when commands are accepted on both inputs
  //  - implement address-bits -> bank selection

  // Subset of DDR3 controller commands/states
  localparam [2:0] ST_IDLE = 3'b111;
  localparam [2:0] ST_READ = 3'b101;
  localparam [2:0] ST_ACTV = 3'b011;
  localparam [2:0] ST_PREC = 3'b010;

  assign bypass = axi_arvalid_i & axi_rready_i | req_q;
  assign cmd_w  = state == ST_IDLE ? CMD_ACTV : cmd_q;

  always @(posedge clock) begin
    if (reset) begin
      state <= ST_IDLE;
      cmd_q <= CMD_NOOP;
      req_q <= 1'b0;
      seq_q <= 1'b0;
    end else begin
      case (state)
        ST_IDLE: begin
          if (axi_arvalid_i && axi_rready_i) begin
            req_q <= 1'b1;
            state <= ST_ACTV;
            req_q <= 1'b1;  // 1st command, RAS#
            cmd_q <= CMD_ACTV;
          end else begin
            state <= ST_IDLE;
            req_q <= ctl_req_i;  // todo: will need extra "skid" logic ??
            cmd_q <= ctl_cmd_i;
            req_q <= 1'b0;
            cmd_q <= CMD_NOOP;
          end
          seq_q <= 1'b0;
        end

        ST_ACTV, ST_READ, ST_PREC: begin
          if (byp_rdy_i) begin
            state <= snext;
            req_q <= req_x;
            seq_q <= seq_x;
            cmd_q <= cmd_x;
          end
        end

        default: begin
          $error("Oh noes");
          state <= ST_IDLE;
          seq_q <= 1'bx;
        end
      endcase
    end
  end


  // -- Acknowledge Signals -- //

  // todo: generate the AXI4 flow-control signals for the read-data path
  always @(posedge clock) begin
    if (!byp_run_i) begin
      arack <= 1'b0;
      rdack <= 1'b0;
    end else begin
      case (state)
        ST_IDLE: {arack, rdack} <= {axi_arvalid_i & axi_rready_i, 1'b0};
        ST_READ: {arack, rdack} <= {1'b0, byp_rdy_i & req_x};  // todo
        default: {arack, rdack} <= 2'b00;
      endcase
    end
  end


  // -- Next-Command Logic -- //

  always @(posedge clock) begin
    if (reset) begin
      snext <= ST_IDLE;
      seq_x <= 1'b0;
      req_x <= 1'b0;
      cmd_x <= CMD_NOOP;
      ba_x  <= 'bx;
      adr_x <= 'bx;
    end else begin
      case (state)
        ST_IDLE: begin
          if (!axi_arvalid_i) begin
            snext <= ST_IDLE;
            seq_x <= 1'b0;
            req_x <= 1'b0;
            cmd_x <= 'bx;
            ba_x  <= 'bx;
            adr_x <= 'bx;
          end else begin
            snext <= ST_READ;
            req_x <= 1'b1;
            seq_x <= ~auto_w;
            cmd_x <= CMD_READ;
            ba_x  <= bank_w;
            adr_x <= col_w;
          end
        end

        ST_ACTV, ST_READ: begin
          if (byp_rdy_i) begin
            snext <= req_s ? ST_READ : ST_IDLE;
            seq_x <= seq_s;
            req_x <= req_s;
            cmd_x <= cmd_s;
            ba_x  <= ba_s;
            adr_x <= col_s;
          end
        end

        default: begin
          if (byp_rdy_i) begin
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
    if (reset) begin
      seq_s <= 1'b0;
      req_s <= 1'b0;
      cmd_s <= 'bx;
      ba_s  <= 'bx;
      col_s <= 'bx;
    end else begin
      case (state)
        ST_IDLE: begin
          seq_s <= ~auto_w;  // todo
          req_s <= 1'b0;
          cmd_s <= 'bx;
          ba_s  <= 'bx;
          col_s <= 'bx;
        end

        ST_ACTV, ST_READ: begin
          // Attempt to queue up another command, if part of a sequence, and
          // the next command is already waiting to be dispatched
          // todo: can this fail, if the 'last' command does not show up, in-
          //   time ??
          if (seq_x) begin
            seq_s <= axi_arvalid_i & axi_rready_i & ~auto_w;  // todo
            req_s <= 1'b1;  // todo
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
      ST_IDLE: dbg_state = !reset ? "IDLE" : "INIT";
      ST_READ: dbg_state = adr_q[10] ? "RD-A" : "RD";
      ST_PREC: dbg_state = "PREC";
      ST_ACTV: dbg_state = "ACT";
      default: dbg_state = "XXX";
    endcase
    case (snext)
      ST_IDLE: dbg_snext = !reset ? "IDLE" : "INIT";
      ST_READ: dbg_snext = adr_q[10] ? "RD-A" : "RD";
      ST_PREC: dbg_snext = "PREC";
      ST_ACTV: dbg_snext = "ACT";
      default: dbg_snext = "XXX";
    endcase
  end

  wire [ 2:0] dbg_cmd_w = byp_rdy_i ? cmd_q : CMD_NOOP;
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


endmodule  // ddr3_bypass