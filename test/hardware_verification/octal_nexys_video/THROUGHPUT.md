# Throughput of the quad APMemory controller (simulation)

Measured 2026-10-31 in cocotb (Icarus) with `OctalMemoryController_quad`, `CHIP_TYPE = 2`, 4x APS12808L model,
50 MHz system clock (25 MHz DDR bus clock), `CEM_MAX_CYCLES = 350` (7 us), fixed latency.

## Method

- Cycles counted on the AXI signals: from the first address handshake (AW or AR) to the last R beat with RLAST or
  the last B response. The cocotbext-axi driver overhead is not included.
- **single**: one burst on its own (latency of one access).
- **8x**: 8 bursts queued back to back (sustained throughput, as a DMA issues them).
- **30 % stall**: the master drops RREADY (reads) or WVALID (writes) in a random 30 % of the cycles.
- Peak of the interface: 4 chips x 8 bit x 2 edges per bus clock = 1 word (4 B) per system clock = 200 MB/s.

Reproduce:

```sh
cd test/cocotb
THROUGHPUT=/tmp/throughput.txt pytest -q test_quad_octal_apmemory_model.py::test_quad_octal_apmemory_model_throughput
```

## Results

| Op | Beats | Bursts | Stall | Cycles | Cycles/burst | Words/cycle | MB/s @ 50 MHz |
|---|---:|---:|---|---:|---:|---:|---:|
| read  |   1 | 1 | -    |    33 |   33.0 | 0.030 |   6.1 |
| read  |   1 | 8 | -    |   272 |   34.0 | 0.029 |   5.9 |
| read  |   1 | 8 | 30 % |   272 |   34.0 | 0.029 |   5.9 |
| read  |   4 | 1 | -    |    36 |   36.0 | 0.111 |  22.2 |
| read  |   4 | 8 | -    |   316 |   39.5 | 0.101 |  20.3 |
| read  |   4 | 8 | 30 % |   555 |   69.4 | 0.058 |  11.5 |
| read  |  16 | 1 | -    |    48 |   48.0 | 0.333 |  66.7 |
| read  |  16 | 8 | -    |   412 |   51.5 | 0.311 |  62.1 |
| read  |  16 | 8 | 30 % |  1899 |  237.4 | 0.067 |  13.5 |
| read  |  64 | 1 | -    |    96 |   96.0 | 0.667 | 133.3 |
| read  |  64 | 8 | -    |   796 |   99.5 | 0.643 | 128.6 |
| read  |  64 | 8 | 30 % |  6078 |  759.8 | 0.084 |  16.8 |
| read  | 256 | 1 | -    |   288 |  288.0 | 0.889 | 177.8 |
| read  | 256 | 8 | -    |  2332 |  291.5 | 0.878 | 175.6 |
| read  | 256 | 8 | 30 % | 23759 | 2969.9 | 0.086 |  17.2 |
| write |   1 | 1 | -    |    24 |   24.0 | 0.042 |   8.3 |
| write |   1 | 8 | -    |   192 |   24.0 | 0.042 |   8.3 |
| write |   1 | 8 | 30 % |   192 |   24.0 | 0.042 |   8.3 |
| write |   4 | 1 | -    |    26 |   26.0 | 0.154 |  30.8 |
| write |   4 | 8 | -    |   208 |   26.0 | 0.154 |  30.8 |
| write |   4 | 8 | 30 % |   274 |   34.2 | 0.117 |  23.4 |
| write |  16 | 1 | -    |    38 |   38.0 | 0.421 |  84.2 |
| write |  16 | 8 | -    |   304 |   38.0 | 0.421 |  84.2 |
| write |  16 | 8 | 30 % |  1030 |  128.8 | 0.124 |  24.9 |
| write |  64 | 1 | -    |    86 |   86.0 | 0.744 | 148.8 |
| write |  64 | 8 | -    |   688 |   86.0 | 0.744 | 148.8 |
| write |  64 | 8 | 30 % |  3042 |  380.2 | 0.168 |  33.7 |
| write | 256 | 1 | -    |   278 |  278.0 | 0.921 | 184.2 |
| write | 256 | 8 | -    |  2224 |  278.0 | 0.921 | 184.2 |
| write | 256 | 8 | 30 % | 12126 | 1515.8 | 0.169 |  33.8 |

## Interpretation

- **Fixed cost per memory transaction:** about 32 cycles for a read and 23 cycles for a write (command, address,
  latency, CE# high). Each additional word costs 1 cycle. Long bursts therefore get close to the 200 MB/s peak,
  short ones are dominated by the fixed cost.
- **Back to back bursts** cost only 0–3.5 cycles more per burst than a single burst. Bursts are not overlapped,
  the controller handles one transaction at a time.
- **Bursts longer than tCEM** (~350 cycles) are split into several memory transactions. This is already included
  in the 256 word rows.
- **Master stalls are expensive.** Every RREADY / WVALID gap ends the memory transaction, and the restart costs the
  full fixed cost again. With 30 % random stalls the throughput drops to ~17 MB/s (read) / ~34 MB/s (write),
  almost independent of the burst length. A small skid buffer would absorb short stalls (open item R3 in
  QUICKSTART.md). Decision postponed until the real stall behaviour of the SoC masters (DC camera, crypto DMA)
  has been measured in chip-di-owas.
- Single beat rows with stall equal the rows without: a one-beat burst has no gap the stall could fall into.
- 30 % random single-cycle stalls are a harsh pattern. Real masters usually stall in longer, rarer stretches
  (FIFO full), which costs less per stalled cycle.

## Typical SoC masters

| Master | Access | Expected (no stalls) |
|---|---|---|
| CPU | single beat, 4 B | ~6 MB/s read, ~8 MB/s write (33 / 24 cycles per access) |
| Crypto DMA | INCR, up to 16 beats | ~62 MB/s read, ~84 MB/s write |
| DC camera | write 16 beats / read 8 beats | ~84 MB/s write, ~40 MB/s read (estimate between the 4 and 16 beat rows) |
