---
title: README for an AXI DDR3 SRAM Memory Controller
author: Patrick Suggate <<patrick.suggate@gmail.com>>
date: 21^st^ October, 2023
colorlinks: true
fontsize: 11pt
---

# Overview of `axi-ddr3-lite`

Note: incomplete, and is currently being developed in another repository (https://github.com/psuggate/misc-verilog-cores).

AXI DDR3 SDRAM Memory Controller for Xilinx GoWin Altera Intel Lattice FPGAs, written in Verilog.

## Design

There are essentially four "layers," somewhat structured like the [OSI model](https://en.wikipedia.org/wiki/OSI_model) -- except for a memory device, so reads and writes are the different "sides" of a transaction, at each level of abstraction. The abstractions can roughly be described as:

1. `TRANSPORT`: AXI4 in this case, and this layer handles alignment, burst-sizes, ordering, and buffering.

2. `NETWORK`: Issues & receives DDR3 memory-controller "packets", and schedules these in order to achieve throughput and latency goals.

3. `DATA LINK`: Low-level DDR3 protocol commands, where all operations are correctly sized (BL8) and meeting the timing requirements of the DDR3 specifications, and the current configuration.

4. `PHYSICAL`: Vendor- and device- specific IO blocks, clock-resources, etc., in order to communicate with a physical DDR3 SDRAM IC (or module).

## Tasks to Complete

These include:

+ synthesise for a GoWin GW2A FPGA, and check area & performance

+ top-level module, with AXI4 data-paths, and testbenches

+ check the timings for all of the supported FSM transitions

Somewhat optional:

+ change the Micro DDR3 simulation model to 1Gb, and check my timings

+ figure out the best way to implement the delay-registers on FPGA's -- counters, or shift-registers?

+ align and "chunk" (to BL8) AXI4 requests

+ optimise the various state-machine encodings, for LUT4-based FPGA implementations

+ AXI4 error-responses for failed transactions

+ ~~remove an additional cycle of latency, between the FSM and the IOB's~~

## DDR3 FSM

Notes:

+ Does not need to know: timings; initialisation details; bus-widths; etc.

+ BUT, for scheduling, knowing the relative costs (when choosing the next command) needs to be known, to ensure good performance; e.g., WRITE -> READ delays are significant (14 cycles, when using: CWL=6, DLL=off, BL8), so alternating between READ & WRITE (for each BL8) will lower throughput by a lot.

+ [OPT] Needs to properly sequence reads and writes, when they refer to the same memory addresses -- this behaviour is not part of AXI4, so do not bother? Read-Before-Write (RBW), Write-Before-Read (WBR), and Write-Before-Write (WBW) are handled above the AXI4 interconnect abstraction layer?

| `curr_state` | event                    | `next_state` |
|:-------------|:-------------------------|:-------------|
| `ST_INIT`    | `ddl_rdy`                | `ST_IDLE`    |
| `ST_IDLE`    | `mem_wrreq OR mem_rdreq` | `ST_ACTV`    |
| `ST_ACTV`    |                          | `ST_WRIT`    |
|              |                          | `ST_READ`    |

## Fast-Path Reads

A dedicated fast-path (user) signal may be useful in situations where multiple functional units share the same memory interface, and there is a clear priority difference between types of accesses. E.g., when the same memory controller provides redraw-data for a display, as well as servicing requests from a microprocessor. Fast-path requests could then allow queue-jumping, for good processor performance, whilst still supporting long burst-transactions, for high thoughput and utilisation.

### Conditions

Can fast-path a READ command if:
 - requested row & bank match the previous row & bank
 - read command FIFO is empty
 - write command FIFO is "low priority"
 - memory-controller is idle, or has just issued the previous read
 - AXI address is appropriately aligned, and the burst-length is supported; OR
 - provide a 'xuser' signal to explicitly request a fast-path operation?

If row and bank do not match, then:
 - looking up the bank in the bank-of-banks will be too slow
 - same bank but different row requires PRECHARGE then ACTIVATE, so _not fast_

Roughly:
```verilog
// Pre-calculate this
always @(posedge clock) begin
  fast_path_read_allowed <= mem_can_read && rcf_empty && wcf_empty;
end

assign addr_is_aligned = araddr[3:0] == 4'b0000; // todo
assign bank_row_match = araddr[19:4] == {prev_bank, prev_row};

always @(posedge clock) begin
  if (arvalid && arready && fast_path_read_allowed && bank_row_match)
  begin
    // fast-path read
  end
end
```
which has a fan-in of around 37 (for 13-bit row addresses). A LUT4-based FPGA could use the following circuit:
 - 8x LUT4's and a carry-chain, for the bank & row comparison;
 - LUT4's to determine if a fast-path read is possible, and requested;
 - LUT4 2:1 MUX with two inputs that determine the 'SEL' value; and
 - DFF to capture the `A[12:0]`{.v} values.
The critical path is through the comparator (approx 5 ns), then the LUT4 2:1 MUX into the DFF (approx 3 ns, including setup & hold times)? Therefore, 125 MHz is possible, which is also the limit of DDR3 DLL=off mode?

### Bypass Port

Todo:
 - takes priority over current RD & WR bursts?
 - how to handle any required ACTIVATE and PRECHARGE bank/row operations?
 - detect if active banks/rows match?

### Future Work

What percentage of READ commands could this be useful for? For a soft-CPU, fast-path reads would be useful for instruction- and data- cache misses? Perhaps much more useful if wrapping-bursts are supported -- so that the cache receives the requested word first, as well?

The READ response path may need to be optimised, for "fast-path responses." Bypassing the read-data FIFO will save two (or more) clock cycles?

## Auto-PRECHARGE

Currently, AUTO-PRECHARGE is used at the end of any request generated by either of the AXI4 data-paths. There is no attempt to see if multiple data-paths are referring to the same page of memory. But multiple (BL8) reads (or, writes) can be performed without additional ACTIVATE's and PRECHARGE's via the use of 'seq' flags, indicating that the the burst-operation is part of a sequence of such, to the same SDRAM page.

Conditions:
 - burst READ or WRITE crosses a page boundary ?
 - REFRESH imminent ?
 - queued READ/WRITE requires a different row, from an active bank ?
