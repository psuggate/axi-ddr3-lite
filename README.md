# axi-ddr3-lite
AXI DDR3 SDRAM Memory Controller for Xilinx GoWin Altera Intel Lattice FPGAs, written in Verilog.

## Design

There are essentially four "layers," somewhat structured like the [OSI model](https://en.wikipedia.org/wiki/OSI_model) -- except that a memory device is a bit like a "buffered, loopback network adapter." The abstractions can roughly be described as:

1. `TRANSPORT`: AXI4 in this case, and this layer handles alignment, burst-sizes, ordering, and buffering.

2. `NETWORK`: Issues & receives DDR3 memory-controller "packets", and schedules these in order to achieve throughput and latency goals.

3. `DATA LINK`: Low-level DDR3 protocol commands, and meeting the timing requirements of the DDR3 specifications, and current DDR3 configuration.

4. `PHYSICAL`: Vendor- and device- specific IO blocks, clock-resources, etc., in order to communicate with a physical DDR3 SDRAM IC (or module).

## DDR3 FSM

Notes:

+ Does not need to know: timings; initialisation details; bus-widths; etc.

+ BUT, for scheduling, knowing the relative costs (when choosing the next command) needs to be known, to ensure good performance; e.g., WRITE -> READ delays are significant (14 cycles, when using: CWL=6, DLL=off, BL8), so alternating between READ & WRITE (for each BL8) will lower throughput by a lot.

+ [OPT] Needs to properly sequence reads and writes, when they refer to the same memory addresses -- this behaviour is not part of AXI4, so do not bother? Read-Before-Write (RBW), Write-Before-Read (WBR), and Write-Before-Write (WBW) are handled above the AXI4 interconnect abstraction layer?
