"""
AXIInterface checks with the legacy MemoryController (INTERFACE_TYPE = "SPI") and no memory model
(test/cocotb_wrapper/Legacy_SPI_AXI_wrapper.v, the SPI data lines are pulled up).

- every AXI read/write burst runs exactly one memory transaction (the legacy SPIController ends a transaction
  without a restart in between)
- RLAST is set on the last beat only, also with PIPELINE_OUTPUT = 1 and RREADY back-pressure
"""

import os
import random
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge
from cocotb_tools.runner import get_runner

IDLE_CYCLES = 1500  # a legacy SPI word takes ~380 cycles; long enough to see a transaction that should not start


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    for name in ["awvalid", "wvalid", "arvalid", "wlast", "rready"]:
        getattr(dut, f"s_axi_{name}").value = 0
    for name in ["awid", "awaddr", "awlen", "awlock", "awcache", "awprot", "wdata", "arid", "araddr", "arlen",
                 "arlock", "arcache", "arprot"]:
        getattr(dut, f"s_axi_{name}").value = 0
    dut.s_axi_awsize.value = 2
    dut.s_axi_arsize.value = 2
    dut.s_axi_awburst.value = 1
    dut.s_axi_arburst.value = 1
    dut.s_axi_wstrb.value = 0xF
    dut.s_axi_bready.value = 1
    dut.reset.value = 1
    await ClockCycles(dut.clk, 5)
    dut.reset.value = 0
    await ClockCycles(dut.clk, 4000)  # configuration write after reset


class Monitor:
    """Counts memory transaction starts and records the R beats."""

    def __init__(self, dut):
        self.dut = dut
        self.starts = 0
        self.beats = []  # (rlast, rresp)
        cocotb.start_soon(self._run())

    def clear(self):
        self.starts = 0
        self.beats = []

    async def _run(self):
        dut = self.dut
        while True:
            await RisingEdge(dut.clk)
            if dut.start_transaction.value:
                self.starts += 1
            if dut.s_axi_rvalid.value and dut.s_axi_rready.value:
                self.beats.append((int(dut.s_axi_rlast.value), int(dut.s_axi_rresp.value)))


async def axi_read(dut, mon, addr, arlen, rready_pattern=None):
    """One AXI read burst; rready_pattern(cycle) -> 0/1 (default: always ready). Returns after the bus is idle."""
    mon.clear()
    await FallingEdge(dut.clk)
    dut.s_axi_araddr.value = addr
    dut.s_axi_arlen.value = arlen
    dut.s_axi_arvalid.value = 1
    while True:
        await RisingEdge(dut.clk)
        if dut.s_axi_arready.value:
            break
    await FallingEdge(dut.clk)
    dut.s_axi_arvalid.value = 0
    cycle = 0
    while len(mon.beats) < arlen + 1 and cycle < 200_000:
        dut.s_axi_rready.value = 1 if rready_pattern is None else rready_pattern(cycle)
        await FallingEdge(dut.clk)
        cycle += 1
    dut.s_axi_rready.value = 0
    await ClockCycles(dut.clk, IDLE_CYCLES)
    assert len(mon.beats) == arlen + 1, f"read @{addr:x} arlen {arlen}: {len(mon.beats)} beats"
    rlast = [b[0] for b in mon.beats]
    assert rlast == [0] * arlen + [1], f"read @{addr:x} arlen {arlen}: rlast per beat {rlast}"
    assert not any(b[1] for b in mon.beats), f"read @{addr:x} arlen {arlen}: rresp {[b[1] for b in mon.beats]}"


async def axi_write(dut, mon, addr, awlen):
    mon.clear()
    await FallingEdge(dut.clk)
    dut.s_axi_awaddr.value = addr
    dut.s_axi_awlen.value = awlen
    dut.s_axi_awvalid.value = 1
    dut.s_axi_wvalid.value = 1
    dut.s_axi_wdata.value = 0x11110000
    dut.s_axi_wlast.value = int(awlen == 0)
    sent = 0
    while sent <= awlen:
        await RisingEdge(dut.clk)
        aw_done = dut.s_axi_awready.value
        w_done = dut.s_axi_wready.value and dut.s_axi_wvalid.value
        await FallingEdge(dut.clk)
        if aw_done:
            dut.s_axi_awvalid.value = 0
        if w_done:
            sent += 1
            dut.s_axi_wdata.value = 0x11110000 + sent
            dut.s_axi_wlast.value = int(sent == awlen)
            if sent > awlen:
                dut.s_axi_wvalid.value = 0
    dut.s_axi_wvalid.value = 0
    await ClockCycles(dut.clk, IDLE_CYCLES)


@cocotb.test()
async def one_transaction_per_burst_test(dut):
    """Every read/write burst runs exactly one memory transaction (review: extra transaction after a read)."""
    await setup(dut)
    mon = Monitor(dut)
    for addr, arlen in [(0x100, 0), (0x200, 3), (0x304, 1)]:
        await axi_read(dut, mon, addr, arlen)
        assert mon.starts == 1, f"read @{addr:x} arlen {arlen}: {mon.starts} memory transactions"
    for addr, awlen in [(0x400, 0), (0x500, 3)]:
        await axi_write(dut, mon, addr, awlen)
        assert mon.starts == 1, f"write @{addr:x} awlen {awlen}: {mon.starts} memory transactions"


@cocotb.test()
async def rlast_with_backpressure_test(dut):
    """RLAST only on the last beat while RREADY stalls (review: PIPELINE_OUTPUT = 1 misaligns RLAST)."""
    await setup(dut)
    mon = Monitor(dut)
    # RREADY low for a long time first, so with the pipeline both output registers hold a beat
    await axi_read(dut, mon, 0x200, 1, rready_pattern=lambda c: int(c > 1200))
    await axi_read(dut, mon, 0x300, 3, rready_pattern=lambda c: int(c > 1200))
    rng = random.Random(1)
    for addr, arlen in [(0x400, 2), (0x500, 5)]:
        await axi_read(dut, mon, addr, arlen, rready_pattern=lambda c: int(rng.random() < 0.3))


def run(pipeline_output):
    proj_path = Path(__file__).resolve().parent
    sources = [
        proj_path / "../../src/SPITransmitter.v",
        proj_path / "../../src/SPIController.v",
        proj_path / "../../src/OctalSPI.v",
        proj_path / "../../src/OSPIController.v",
        proj_path / "../../src/AXIInterface.v",
        proj_path / "../../src/MemoryController.v",
        proj_path / "../../test/cocotb_wrapper/Legacy_SPI_AXI_wrapper.v",
    ]
    parameters = {"PIPELINE_OUTPUT": pipeline_output}
    sim = os.getenv("SIM", "icarus")
    runner = get_runner(sim)
    build_dir = proj_path / f"sim_build_legacy_spi_pipe{pipeline_output}"
    runner.build(
        sources=sources,
        hdl_toplevel="Legacy_SPI_AXI_wrapper",
        build_args=["-g2012"] if sim == "icarus" else [],
        build_dir=build_dir,
        always=True,
        parameters=parameters,
    )
    runner.test(
        hdl_toplevel="Legacy_SPI_AXI_wrapper",
        test_module="test_axi_interface_legacy_spi",
        build_dir=build_dir,
        parameters=parameters,
    )


def test_axi_interface_legacy_spi():
    run(pipeline_output=0)


def test_axi_interface_legacy_spi_pipeline_output():
    run(pipeline_output=1)


if __name__ == "__main__":
    run(pipeline_output=int(os.getenv("PIPELINE_OUTPUT", "0")))
