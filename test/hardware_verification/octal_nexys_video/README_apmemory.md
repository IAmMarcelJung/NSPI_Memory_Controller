# Quad APMemory controller: simulation and Nexys Video emulation

`OctalMemoryController_quad` with `CHIP_TYPE = 2` (4x APS12808L on the FMC PSRAM board), 50 MHz system clock,
25 MHz bus clock.

## Files

| File | What |
|---|---|
| `test/sim_models/aps12808l_model.v` | Behavioral APS12808L model (written from datasheet Rev 1.5, no vendor model exists). Commands 00h/80h/20h/A0h/40h/C0h/FFh, latency (variable / fixed, optional random refresh pushout), wrap, DM masking. Counts protocol errors (tRP, tRST, tCPH, tRC, tCEM, odd start address, < 2 byte write, reserved latency codes) in `error_count`, unknown commands in `unknown_cmd_count`. No DQ setup/hold checks. |
| `test/cocotb_wrapper/Quad_Octal_Memory_Controller_apm_wrapper.v` | Controller (CHIP_TYPE 2) + 4 models, RESET# driven by the controller like on the board. `MEMORY_RESET_CYCLES` = 20000 to keep the simulation short. |
| `test/cocotb/test_quad_octal_apmemory_model.py` | cocotb tests (Icarus). Mapping tests use the backdoor `gen_memory[i].Memory.mem[]` and expect AXI byte address A in chip A % 4, byte A / 4. |
| `top_quad.v` | FPGA top: MMCM 100 -> 50 MHz, JTAG-AXI master, controller, IO buffers. |
| `build_quad.tcl` | Non-project Vivado build incl. the clk_wiz / jtag_axi IPs -> `build/top_quad.bit`, `build/timing_summary.rpt`. |
| `run_hw_scenario.tcl`, `hw_scenario.py` | One fixed AXI sequence that runs on the board (JTAG-AXI) and in simulation, results are compared word by word. |
| `stress_test.tcl` | Board stress test: toggling / walking / random patterns over 64 KB, errors per chip, re-reads on mismatch. |
| `slew_variants.tcl` | Bitstreams with DRIVE 4/8/12/16 x SLEW SLOW/FAST on the PSRAM clock (slew margin test). |
| `Nexys-Video-Master_Quad.xdc`, `Timing_RAM_[A-D].xdc` | Pinout (matches Timo's gist) and IO timing. |

## Simulation

Needs Icarus Verilog >= 12 (tested 13.0), Python with cocotb 2.0 (tested 2.0.1), cocotbext-axi (0.1.28), pytest.
With the nix flake of the repo (`flake.nix`, currently uncommitted in the main checkout):

```sh
nix develop <repo with flake.nix> --command bash
cd test/cocotb
pytest -v test_quad_octal_apmemory_model.py                          # all tests, ~1 min
TEST_FILTER=byte_strobe_test pytest test_quad_octal_apmemory_model.py::test_quad_octal_apmemory_model
TEST_FILTER=row_boundary_test WAVES=1 pytest ...::test_quad_octal_apmemory_model   # FST in sim_build_apmemory_0_-1/
TRACE=402000,405000 TEST_FILTER=...  pytest ...                       # print controller states per clock in a time window (ns)
```

Without nix: `pip install cocotb==2.0.1 cocotbext-axi pytest` and `iverilog` on PATH, same commands.

Expected: all tests pass. The pytest entries:
- `test_quad_octal_apmemory_model`: all cocotb tests, CE# limit 350 cycles (7 us)
- `test_quad_octal_apmemory_model_short_cem`: data and stall tests with a CE# limit of 60 cycles, so almost every
  burst is split by the tCEM counter
- `test_quad_octal_apmemory_model_refresh_pushout`: model with refresh collisions on 50 % of variable latency reads
- `test_quad_octal_apmemory_model_hw_scenario`: only runs with `HW_SCENARIO=<result file>` set

Note for own tests: cocotbext-axi `clear_pause_generator()` keeps the last pause state, set
`channel.pause = False` afterwards, and use endless pause generators (`itertools.cycle`).

## Emulation on the Nexys Video

Vivado 2025.1, board + FMC PSRAM board connected, SW0 (reset) off.

```sh
cd test/hardware_verification/octal_nexys_video
VIVADO=/home/marcel/Programs/Vivado/2025.1/Vivado/bin/vivado
$VIVADO -mode batch -source build_quad.tcl                            # ~6 min, check build/timing_summary.rpt

# scenario on the board and in the simulation, then compare
python3 hw_scenario.py tcl > build/hw_scenario.tcl
$VIVADO -mode batch -source run_hw_scenario.tcl -tclargs program      # -> build/hw_result.txt
(cd ../../cocotb && HW_SCENARIO=/tmp/sim_result.txt pytest test_quad_octal_apmemory_model.py::test_quad_octal_apmemory_model_hw_scenario)
python3 hw_scenario.py compare build/hw_result.txt /tmp/sim_result.txt   # "0 mismatching words"

# stress test: <bitfile or - for the already programmed one> <passes> <result file>
$VIVADO -mode batch -source stress_test.tcl -tclargs build/top_quad.bit 10 build/stress.txt
```

Interactive alternative: open the hardware manager in the GUI, program `build/top_quad.bit`, then in the Tcl console:

```tcl
create_hw_axi_txn -force w [get_hw_axis hw_axi_1] -type write -address 00000100 -len 4 -data 44444444333333332222222211111111
run_hw_axi [get_hw_axi_txns w]
create_hw_axi_txn -force r [get_hw_axis hw_axi_1] -type read -address 00000100 -len 4
run_hw_axi [get_hw_axi_txns r]; get_property DATA [get_hw_axi_txns r]   ;# first beat is the rightmost word
```

Notes:
- AXI stalls (WVALID low, RREADY low) and bursts longer than `CEM_MAX_CYCLES` end the memory transaction (CE# high)
  and restart it at the next beat. Very sparse RREADY (e.g. 8 of 200 cycles) gives about one word per restart.
- Supported AXI bursts: INCR with full width beats (AxSIZE = 2), and single beats of any burst type and size (byte
  lanes via WSTRB). FIXED / WRAP bursts and narrow bursts with more than one beat, and beats wider than the bus, are
  not executed: the memory is not accessed, the write gets BRESP = SLVERR, the read returns all beats with zero data
  and RRESP = SLVERR.
- Reset: `rstn` is active low with asynchronous assert and synchronous release (SoC: `resetn_async`), all flip-flops
  have an asynchronous reset.
- Clock range `CLK_FREQ_MIN_HZ` .. `CLK_FREQ_HZ` (default 10 .. 50 MHz): the controller counts clk cycles, it has no
  time reference. Minimum times come from the highest frequency: RESET# low 20 ms (`MEMORY_RESET_CYCLES`), tRST wait
  4 us. The maximum time comes from the lowest frequency: CE# low at most 7 us (`CEM_MAX_CYCLES`, 70 cycles), tCEM is
  8 us. Any clk in the range is safe, a slower clk only makes the reset wait longer. Both cycle counts can still be
  overridden. Cost of the 10 MHz floor at 50 MHz: bursts longer than about 40 beats are split into several memory
  transactions (256 beats: read 113 instead of 176 MB/s, write 140 instead of 184 MB/s); 16-beat bursts and single
  accesses are not affected. Verified in sim at 50 and 10 MHz; the fixed protocol settings (latency code 5, bus
  clock = clk / 2) work at any clk in the range.
- `BUS_CLOCK_NEGEDGE` (default 1): the bus clock comes from a negedge register, so write data and CE# are centre
  aligned to it (half a clk period setup and hold) and read data is captured on the negedge. 0 gives the old edge
  aligned bus clock, which has negative write hold in the ASIC STA. Nexys: `BUS_CLOCK_NEGEDGE=0` for build_quad.tcl
  builds that variant into `build_posedge/`.
- The controller holds RESET# of the memories low for 20 ms after reset, then writes MR0 (fixed latency). AXI accesses are held off (busy) until then.
- The Digilent adapter can show up as two JTAG targets, the scripts pick the one with the FPGA.
- `axi_verification_test.tcl` (older) still has expectations from the old address mapping.
