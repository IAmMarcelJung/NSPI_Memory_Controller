# Quickstart: quad APMemory controller (sim + Nexys Video)

Details: [README_apmemory.md](README_apmemory.md)

## Simulation (~1 min, all tests pass)

Needs Icarus 13, cocotb 2.0.1, cocotbext-axi 0.1.28, pytest (or the repo's nix flake).

```sh
nix develop <checkout with flake.nix> --command bash   # or a venv with the packages above
cd test/cocotb
pytest -v test_quad_octal_apmemory_model.py
```

## Board (Nexys Video + FMC PSRAM board, SW0 off)

```sh
cd test/hardware_verification/octal_nexys_video
VIVADO=/home/marcel/Programs/Vivado/2025.1/Vivado/bin/vivado

$VIVADO -mode batch -source build_quad.tcl                         # bitstream, ~6 min
# negedge bus clock variant: BUS_CLOCK_NEGEDGE=1 $VIVADO ... (output in build_negedge/)

python3 hw_scenario.py tcl > build/hw_scenario.tcl
$VIVADO -mode batch -source run_hw_scenario.tcl -tclargs program   # board -> build/hw_result.txt
(cd ../../cocotb && HW_SCENARIO=/tmp/sim.txt \
    pytest test_quad_octal_apmemory_model.py::test_quad_octal_apmemory_model_hw_scenario)
python3 hw_scenario.py compare build/hw_result.txt /tmp/sim.txt   # expect "0 mismatching words"

$VIVADO -mode batch -source stress_test.tcl -tclargs - 10 build/stress.txt   # expect 0 errors
```

The FPGA sits on JTAG target `...BDB` (FTDI channel B); the scripts try every target, so the
"No devices detected" error for `...BD` is expected.

## Files to review

| File | Change |
|---|---|
| `src/AXIInterface.v` | one transaction at a time, rlast fix, wready only in the burst, restarts, read drop, BRESP after write, a waiting AW without W no longer blocks reads (DMA copy deadlock), SLVERR for unsupported bursts (FIXED/WRAP/narrow with more than one beat) |
| `src/OSPIController_quad.v` | RESET#/tRST + MR0 init, no write enable for APMemory, tCEM counter, write cut on stall |
| `src/OctalSPI_quad.v` | chip address = AXI/4, read pad drop, fixed read latency, MR write latency 1, test parameter `BUS_CLOCK_NEGEDGE` |
| `src/OctalMemoryController_quad.v` | parameters `MEMORY_RESET_CYCLES`, `CEM_MAX_CYCLES`, `BUS_CLOCK_NEGEDGE`, wiring |
| `test/sim_models/aps12808l_model.v` | APS12808L behavioral model |
| `test/cocotb/test_quad_octal_apmemory_model.py`, `test/cocotb_wrapper/Quad_Octal_Memory_Controller_apm_wrapper.v` | tests + wrapper |
| `test/hardware_verification/octal_nexys_video/` | `top_quad.v` fixes, build / scenario / stress / slew scripts |

## Open decisions (owner in brackets)

### RTL (current focus)

R1. **Write hold fix for the ASIC** [user/Timo]: ASIC STA has write hold −4.7/−3.2 ns with the posedge bus clock
    (data and clock edge aligned). Option (a, RTL): negedge bus clock, parameter `BUS_CLOCK_NEGEDGE=1` (test build
    exists, default 0; read data is then captured on the negedge, no latency change). Alternatives outside the RTL:
    (b) deliberate output delay on the data pads (see P1), (c) PCB trace length rule (see B2). Nexys: negedge build
    passes sim and the board sweep (0 errors at DRIVE 8/12/16, write hold +4.1..+6.2 ns at all settings vs.
    −1.3..+0.8 ns with posedge). Decision = which default `BUS_CLOCK_NEGEDGE` gets.
R2. **Clock frequency assumptions** [user]: `CEM_MAX_CYCLES` (350 = 7 us), `MEMORY_RESET_CYCLES` (20 ms) and the
    tRST wait (200 cycles = 4 us) assume 50 MHz: derive them from a clock frequency parameter, or document it.
R3. **Read throughput under stalls** [user]: restart on stall has no skid buffer, a very sparse RREADY
    (e.g. 8 of 200 cycles) gives about one word per restart. Add a small skid buffer if a real master behaves like
    that (depends on the toplevel AXI master, see P5).

### Tests and Nexys setup

T1. **Old tests** [Timo]: the Infineon tests (`test_quad_octal_memory_controller_model.py`) expect the old address
    mapping and can't run without the vendor model; `axi_verification_test.tcl` also expects the old mapping.
T2. **Nexys XDC comments** [user]: `Timing_RAM_*.xdc` / `Nexys-Video-Master.xdc` still say negedge / centre aligned,
    the default RTL is posedge / edge aligned (correct again if the negedge variant is chosen in R1).
T3. **Scope measurement** [user]: CLK edge at the farthest PSRAM ball on the Nexys, to confirm the IBIS estimates
    (12 mA SLOW est. 3.4/2.6 ns, 8 mA SLOW 4.6/3.6 ns). Input for M1.

### ASIC pads, toplevel and SDC

P1. **Pad variant** [user, toplevel]: one Out30mA clock pad per PSRAM (4 instead of 1, agreed in principle),
    IOPadInOut30mA for DQ/DQS (recommended). SDC section 8 in `constraints/psram_interface.sdc` not active yet.
P2. **IOVdd/IOVss pairs in the PSRAM pad group** [toplevel]: count unknown, check the IHP pad ring guidelines
    (rule of thumb 1 pair per 4–8 strong outputs → ~6–10 pairs). Group pads per chip (CLK next to its CE#/DQS/DQ).
P3. **Package** [toplevel]: not chosen, the package delay in the SDC (0.1–0.5 ns) is a guess.
P4. **SDC housekeeping** [other session, needs user OK]: RESET# max_transition waiver note; TODO 2/4 (pad names,
    clock uncertainty) as notes for the toplevel integrator.
P5. **Toplevel AXI master** [toplevel]: burst lengths and RREADY/WVALID behaviour decide whether R3 is needed.

### PCB

B1. **PCB rules** [user/board designer]: CE# pull-ups and CLK pull-downs (datasheet section 7: CE# high, CLK low
    during power-up, the IO lib has no pull pads), PSRAM VDDQ on the same 3.3 V rail as the chip IOVdd
    (VIH ≥ VDDQ − 0.4 V), loads ≤ ~16 pF per pin, traces 0.1–0.5 ns.
B2. **Trace length rule** [user/board designer]: only needed if R1 is solved on the board (option c).

### Memory vendor

M1. **tKHKL** [user → AP Memory]: datasheet max 1.0 ns (-7) / 1.2 ns (-9), our ASIC pads ~2.5 ns (slow corner).
    Ask AP Memory: "Is there a frequency-independent minimum CLK slew rate (V/ns)? Is ~2.5 ns (10–90 %) at 25 MHz,
    3 V VDDQ acceptable?" Nexys: error-free up to ~4.6/3.6 ns (IBIS estimate, DRIVE 8 SLOW). DRIVE 4 (~6–7 ns) fails
    even with positive hold (negedge build): random bit flips on chips C/D, mostly read side, mainly the 55/AA pattern
    → the slow edge itself is the limit somewhere between ~4.6 and ~6–7 ns on these parts.

## Decided (2026-10-30)

- sys_clk 50 MHz, bus clock 25 MHz, room temperature only → tCEM 8 us.
- Controller integrated flat into the toplevel, no macro-level budgets.
- RESET#: keep the pin, one pad for all 4 chips + slew waiver; the controller holds RESET# low for
  `MEMORY_RESET_CYCLES` (20 ms), waits tRST (4 us), writes MR0 (fixed latency). No Global Reset.
- AXI stall fix: restart on stall + CE#-low counter (`CEM_MAX_CYCLES`, 7 us). Not clock stop with CE# low.
- Address mapping: AXI byte address A → chip A % 4, chip byte A / 4.
- Clock pads at 30 mA (16 mA would exceed the 3.5 ns lib max_transition at ~12 pF).
- The datasheet PDF is not committed (no distribution rights).
- `` `default_nettype `` leak from `3ff5ccd`: `AXIInterface.v` restores `` `default_nettype wire `` at the end (`eab141d`).
