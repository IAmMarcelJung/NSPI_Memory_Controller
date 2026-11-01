"""
Tests for OctalMemoryController_quad with CHIP_TYPE = 2 (APMemory APS12808L), the tapeout/hardware setting.

The memory is the behavioral model test/sim_models/aps12808l_model.v, so this runs in Icarus Verilog.
Every test also checks the protocol error counter of the four models.

Address mapping expected by the backdoor tests: the four chips share one address, each AXI word holds one byte
per chip, so AXI byte address A lives in chip byte A // 4, byte lane A % 4 is chip A % 4.
"""

import itertools
import os
import random
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, First, RisingEdge, Timer
from cocotb.utils import get_sim_time
from cocotb_tools.runner import get_runner
from cocotbext.axi import AxiBurstType, AxiBus, AxiMaster, AxiResp

# 50 MHz system clock, 25 MHz bus clock; run(clk_period_ns=...) sets a slower clock through the environment
CLK_PERIOD_NS = int(os.getenv("CLK_PERIOD_NS", "20"))
NUM_CHIPS = 4
T_RST_NS = 2000  # reset to command valid
TIMEOUT_US = 500


def models(dut):
    return [dut.gen_memory[i].Memory for i in range(NUM_CHIPS)]


def model_errors(dut):
    return [int(m.error_count.value) for m in models(dut)]


def unknown_commands(dut):
    return [int(m.unknown_cmd_count.value) for m in models(dut)]


def mem_get(dut, axi_addr, num_bytes):
    """Read the model memories through the backdoor with the expected address mapping."""
    data = []
    for a in range(axi_addr, axi_addr + num_bytes):
        data.append(int(dut.gen_memory[a % NUM_CHIPS].Memory.mem[a // NUM_CHIPS].value))
    return data


def mem_set(dut, axi_addr, data):
    for i, value in enumerate(data):
        a = axi_addr + i
        dut.gen_memory[a % NUM_CHIPS].Memory.mem[a // NUM_CHIPS].value = value


def mem_fill(dut, value, num_bytes):
    for m in models(dut):
        for i in range(num_bytes):
            m.mem[i].value = value


async def trace(dut, start_ns, end_ns):
    """Debug: print the controller state every clock cycle in a time window (env TRACE=start_ns,end_ns)."""
    c = dut.Controller
    sig = {
        "ctl": c.SPI_Controller.control_state_reg,
        "spi": c.SPI_Controller.SPI_Transmitter.state_reg,
        "cnt": c.SPI_Controller.SPI_Transmitter.count_reg,
        "pad": c.SPI_Controller.SPI_Transmitter.is_transfer_front_padded_reg,
        "lw_spi": c.SPI_Controller.SPI_Transmitter.last_word_reg,
        "lw_ctl": c.SPI_Controller.last_word_reg,
        "rx": c.SPI_Controller.SPI_Transmitter.recieved_next_word_reg,
        "rd_st": c.AXI_Interface.read_state_reg,
        "drop": c.AXI_Interface.read_drop_reg,
        "rdy": c.AXI_Interface.i_read_word_ready,
        "rr": dut.s_axi_rready,
        "busy": c.spi_busy,
        "wv": dut.s_axi_wvalid,
        "wr": dut.s_axi_wready,
        "wl": dut.s_axi_wlast,
        "wstop": c.SPI_Controller.write_stop_reg,
        "wcnt": c.AXI_Interface.write_count_reg,
        "bv": dut.s_axi_bvalid,
        "rd_cnt": c.AXI_Interface.read_count_reg,
        "wr_st": c.AXI_Interface.write_state_reg,
        "arv": dut.s_axi_arvalid,
        "arr": dut.s_axi_arready,
        "rv": dut.s_axi_rvalid,
        "rl": dut.s_axi_rlast,
        "cs": dut.gen_memory[0].Memory.CE_n,
    }
    await Timer(start_ns, unit="ns")
    while get_sim_time("ns") < end_ns:
        await RisingEdge(dut.clk)
        dut._log.info("TRACE " + " ".join(f"{k}={v.value}" for k, v in sig.items()))


async def setup(dut, wait_for_memory=True):
    """Start the clock, reset the controller and wait until the memory may be accessed."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, "ns").start())
    if os.getenv("TRACE"):
        cocotb.start_soon(trace(dut, *[int(x) for x in os.environ["TRACE"].split(",")]))
    dut.test_number.value = dut.test_number.value + 1
    axi = AxiMaster(AxiBus.from_prefix(dut, "s_axi"), dut.clk, dut.reset)

    dut.reset.value = 1
    await ClockCycles(dut.clk, 10)
    dut.reset.value = 0
    await ClockCycles(dut.clk, 2)

    if wait_for_memory:
        # the controller releases RESET# of the memory by itself, after that tRST has to pass
        if dut.spi_reset_neg.value != 1:
            await RisingEdge(dut.spi_reset_neg)
        await Timer(T_RST_NS + 100, unit="ns")
    return axi


async def with_timeout(coro, timeout_us=TIMEOUT_US):
    task = cocotb.start_soon(coro)
    # the timeouts are given for 50 MHz, a slower clock takes proportionally longer
    timeout = Timer(timeout_us * CLK_PERIOD_NS / 20, unit="us")
    trigger = await First(task, timeout)
    assert trigger is not timeout, "AXI transaction timed out"
    return task.result()


async def wait_idle(dut):
    while dut.Controller.spi_busy.value == 1:
        await RisingEdge(dut.clk)
    await Timer(100, unit="ns")


def check_no_model_errors(dut, before):
    after = model_errors(dut)
    assert after == before, f"memory model reported protocol errors: before {before}, after {after}"


def pattern(num_bytes, seed):
    rnd = random.Random(seed)
    return bytes(rnd.randrange(256) for _ in range(num_bytes))


# --------------------------------------------------------------------------------------------------------------
@cocotb.test()
async def reset_sequence_test(dut):
    """RESET# must be low >= tRP and the controller must not accept a transaction before tRST has passed."""
    axi = await setup(dut, wait_for_memory=False)

    assert dut.spi_reset_neg.value == 0
    t_start = get_sim_time("ns")
    await RisingEdge(dut.spi_reset_neg)
    t_release = get_sim_time("ns")
    assert t_release - t_start >= 1000, "tRP: RESET# low pulse shorter than 1 us"

    # an AXI write issued right after RESET# released must not reach the memory before tRST
    before = model_errors(dut)
    await with_timeout(axi.write(0x0, b"\x11\x22\x33\x44"))
    await wait_idle(dut)
    check_no_model_errors(dut, before)


@cocotb.test()
async def no_unknown_commands_test(dut):
    """The APMemory command set has no write enable (06h) - the controller must not send unknown commands."""
    axi = await setup(dut)
    before = unknown_commands(dut)
    await with_timeout(axi.write(0x100, pattern(8, 1)))
    await wait_idle(dut)
    await with_timeout(axi.read(0x100, 8))
    await wait_idle(dut)
    after = unknown_commands(dut)
    assert after == before, f"unknown commands sent to the memory: {[a - b for a, b in zip(after, before)]}"


@cocotb.test()
@cocotb.parametrize(params=[(8, 0x0000), (16, 0x0100), (64, 0x0200), (256, 0x0400), (1024, 0x0800)])
async def write_read_roundtrip_test(dut, params):
    """Aligned burst write followed by a burst read of the same data (frontdoor only)."""
    num_bytes, addr = params
    axi = await setup(dut)
    before = model_errors(dut)

    data = pattern(num_bytes, addr)
    await with_timeout(axi.write(addr, data))
    await wait_idle(dut)
    result = await with_timeout(axi.read(addr, num_bytes))
    await wait_idle(dut)

    assert result.data == data
    check_no_model_errors(dut, before)


@cocotb.test()
@cocotb.parametrize(params=[(8, 0x0000), (64, 0x0200), (1024, 0x0800)])
async def write_address_mapping_test(dut, params):
    """Burst write, then check where the bytes ended up in the chips (backdoor)."""
    num_bytes, addr = params
    axi = await setup(dut)
    mem_fill(dut, 0x00, 0x1000)

    data = pattern(num_bytes, addr + 1)
    await with_timeout(axi.write(addr, data))
    await wait_idle(dut)

    assert mem_get(dut, addr, num_bytes) == list(data)


@cocotb.test()
@cocotb.parametrize(params=[(8, 0x0000), (64, 0x0200), (1024, 0x0800)])
async def read_address_mapping_test(dut, params):
    """Preload the chips (backdoor), then burst read."""
    num_bytes, addr = params
    axi = await setup(dut)

    data = pattern(num_bytes, addr + 2)
    mem_set(dut, addr, data)
    result = await with_timeout(axi.read(addr, num_bytes))
    await wait_idle(dut)

    assert result.data == data


@cocotb.test()
async def single_word_consistency_test(dut):
    """
    Single word writes at scattered addresses must be readable as single words and as one burst.
    Mapping independent: catches aliasing between burst offsets and start addresses.
    """
    axi = await setup(dut)
    before = model_errors(dut)
    base = 0x1000
    num_words = 16

    await with_timeout(axi.write(base, bytes(4 * num_words)))
    await wait_idle(dut)

    rnd = random.Random(7)
    expected = bytearray(4 * num_words)
    for word in rnd.sample(range(num_words), num_words):
        value = rnd.randrange(1 << 32).to_bytes(4, "little")
        expected[4 * word : 4 * word + 4] = value
        dut._log.info(f"single write word {word}")
        await with_timeout(axi.write(base + 4 * word, value))
        await wait_idle(dut)

    for word in range(num_words):
        dut._log.info(f"single read word {word}")
        result = await with_timeout(axi.read(base + 4 * word, 4))
        await wait_idle(dut)
        assert result.data == expected[4 * word : 4 * word + 4], f"single read of word {word}"

    result = await with_timeout(axi.read(base, 4 * num_words))
    await wait_idle(dut)
    assert result.data == bytes(expected), "burst read of all words"
    check_no_model_errors(dut, before)


@cocotb.test()
async def row_boundary_test(dut):
    """
    A legal AXI burst (inside a 4 KB page) that covers 1K chip bytes or more must not wrap inside the memory.
    The linear burst commands (20h / A0h) wrap at 1K chip bytes.
    """
    axi = await setup(dut)
    addr = 0x2300
    num_bytes = 1024

    data = pattern(num_bytes, 3)
    await with_timeout(axi.write(addr, data))
    await wait_idle(dut)

    for offset in range(0, num_bytes, 128):
        result = await with_timeout(axi.read(addr + offset, 4))
        await wait_idle(dut)
        assert result.data == data[offset : offset + 4], f"word at offset {offset:#x}"


@cocotb.test()
@cocotb.parametrize(params=[(0x0104, 4), (0x0104, 8), (0x0100, 12), (0x0104, 12)])
async def unaligned_word_test(dut, params):
    """Bursts starting on an odd word and/or with an odd word count (the chips need pairs of bytes)."""
    addr, num_bytes = params
    axi = await setup(dut)
    before = model_errors(dut)

    await with_timeout(axi.write(addr - 8, b"\xff" * (num_bytes + 16)))
    await wait_idle(dut)

    data = pattern(num_bytes, addr + num_bytes)
    await with_timeout(axi.write(addr, data))
    await wait_idle(dut)

    result = await with_timeout(axi.read(addr - 8, num_bytes + 16))
    await wait_idle(dut)
    assert result.data == b"\xff" * 8 + data + b"\xff" * 8
    check_no_model_errors(dut, before)


@cocotb.test()
async def byte_strobe_test(dut):
    """Narrow writes with WSTRB must only change the selected bytes."""
    axi = await setup(dut)
    before = model_errors(dut)
    addr = 0x0300

    await with_timeout(axi.write(addr, b"\x00" * 8))
    await wait_idle(dut)
    await with_timeout(axi.write(addr + 1, b"\xaa\xbb"))  # WSTRB 0b0110
    await wait_idle(dut)
    await with_timeout(axi.write(addr + 7, b"\xcc"))  # WSTRB 0b1000 in the second word
    await wait_idle(dut)

    result = await with_timeout(axi.read(addr, 8))
    await wait_idle(dut)
    assert result.data == b"\x00\xaa\xbb\x00\x00\x00\x00\xcc"
    check_no_model_errors(dut, before)


@cocotb.test()
async def back_to_back_test(dut):
    """Writes and reads issued without idle time in between (tCPH / tRC between transactions)."""
    axi = await setup(dut)
    before = model_errors(dut)

    writes = [cocotb.start_soon(axi.write(0x0400 + 32 * i, pattern(32, i))) for i in range(4)]
    for w in writes:
        await with_timeout(w)
    reads = [cocotb.start_soon(axi.read(0x0400 + 32 * i, 32)) for i in range(4)]
    for i, r in enumerate(reads):
        result = await with_timeout(r)
        assert result.data == pattern(32, i), f"read {i}"
    await wait_idle(dut)
    check_no_model_errors(dut, before)


@cocotb.test()
async def read_backpressure_test(dut):
    """The AXI master stalls RREADY - no read data may be lost."""
    axi = await setup(dut)
    data = pattern(64, 11)
    await with_timeout(axi.write(0x0500, data))
    await wait_idle(dut)

    axi.read_if.r_channel.set_pause_generator(itertools.cycle([1, 1, 0, 1, 0, 0, 0, 1]))
    result = await with_timeout(axi.read(0x0500, 64))
    axi.read_if.r_channel.clear_pause_generator()
    axi.read_if.r_channel.pause = False  # clear keeps the last pause state
    await wait_idle(dut)
    assert result.data == data


@cocotb.test()
async def write_backpressure_test(dut):
    """The AXI master inserts WVALID gaps - the controller must not write stale data."""
    axi = await setup(dut)
    data = pattern(64, 12)

    axi.write_if.w_channel.set_pause_generator(itertools.cycle([0, 1, 1, 0, 0, 1, 0, 0]))
    await with_timeout(axi.write(0x0600, data))
    axi.write_if.w_channel.clear_pause_generator()
    axi.write_if.w_channel.pause = False  # clear keeps the last pause state
    await wait_idle(dut)

    result = await with_timeout(axi.read(0x0600, 64))
    await wait_idle(dut)
    assert result.data == data


# --------------------------------------------------------------------------------------------------------------
# Traffic of the SoC masters (chip-di-owas): all go through a verilog-axi crossbar, which sets the ID to
# master ID | port << 4 and routes B / R back by ID. The crypto core uses verilog-axi axi_dma (read -> AES ->
# write, bursts of up to 16 beats), and axi_dma_wr raises AWVALID of later bursts before it has the write data.


@cocotb.test()
async def read_while_write_waits_for_data_test(dut):
    """
    AW is pending without write data, and the write data depends on a read issued afterwards (DMA copy).
    The read must not wait for the write, otherwise the system deadlocks.
    """
    axi = await setup(dut)
    before = model_errors(dut)
    src = pattern(64, 21)
    await with_timeout(axi.write(0x0800, src))
    await wait_idle(dut)

    axi.write_if.w_channel.pause = True
    write = cocotb.start_soon(axi.write(0x0900, bytes(b ^ 0x5A for b in src)))
    await ClockCycles(dut.clk, 10)
    assert dut.s_axi_awvalid.value == 1 and dut.s_axi_wvalid.value == 0
    try:
        result = await with_timeout(axi.read(0x0800, 64), timeout_us=100)
    finally:
        axi.write_if.w_channel.pause = False
    assert result.data == src
    await with_timeout(write)
    await wait_idle(dut)

    result = await with_timeout(axi.read(0x0900, 64))
    await wait_idle(dut)
    assert result.data == bytes(b ^ 0x5A for b in src)
    check_no_model_errors(dut, before)


@cocotb.test()
@cocotb.parametrize(seed=[1, 2])
async def dma_copy_test(dut, seed):
    """
    Copy like the crypto core's axi_dma: 16 beat bursts, AW of each write burst is sent before the read burst
    it depends on, W only after that read data arrived. Random RREADY / WVALID stalls, crossbar style IDs.
    """
    axi = await setup(dut)
    before = model_errors(dut)
    rnd = random.Random(seed)
    src_addr, dst_addr, bursts = 0x2000, 0x3000, 8
    src = pattern(64 * bursts, 30 + seed)
    await with_timeout(axi.write(src_addr, src))
    await wait_idle(dut)

    def pauses():
        while True:
            yield rnd.random() < 0.3

    for k in range(bursts):
        chunk = src[64 * k : 64 * (k + 1)]
        axi.write_if.w_channel.pause = True
        write = cocotb.start_soon(axi.write(dst_addr + 64 * k, bytes(b ^ 0xA5 for b in chunk), awid=0x40))
        await ClockCycles(dut.clk, rnd.randrange(1, 8))
        axi.read_if.r_channel.set_pause_generator(pauses())
        try:
            result = await with_timeout(axi.read(src_addr + 64 * k, 64, arid=0x40), timeout_us=100)
        finally:
            axi.read_if.r_channel.clear_pause_generator()
            axi.read_if.r_channel.pause = False  # clear keeps the last pause state
            axi.write_if.w_channel.set_pause_generator(pauses())
        assert result.data == chunk, f"read burst {k}"
        await with_timeout(write)
        axi.write_if.w_channel.clear_pause_generator()
        axi.write_if.w_channel.pause = False
    await wait_idle(dut)

    result = await with_timeout(axi.read(dst_addr, 64 * bursts))
    await wait_idle(dut)
    assert result.data == bytes(b ^ 0xA5 for b in src)
    check_no_model_errors(dut, before)


@cocotb.test()
async def axi_id_test(dut):
    """BID / RID must return the ID of the request (cocotbext-axi asserts on an unexpected ID)."""
    axi = await setup(dut)
    before = model_errors(dut)
    for n, axi_id in enumerate([0x00, 0x43, 0x30, 0x11, 0x20, 0xFF, 0x80]):
        addr = 0x5000 + 0x40 * n
        data = pattern(32, 40 + n)
        await with_timeout(axi.write(addr, data, awid=axi_id))
        result = await with_timeout(axi.read(addr, 32, arid=axi_id))
        assert result.data == data, f"ID {axi_id:#x}"
    await wait_idle(dut)
    check_no_model_errors(dut, before)


@cocotb.test()
async def queued_bursts_with_stall_test(dut):
    """
    Several bursts queued back to back while the master stalls RREADY / WVALID. The next address is accepted while
    the last beat of the previous burst still waits in the output register, that beat must keep its rlast
    (cocotbext-axi asserts on a missing rlast) and every burst its own data.
    """
    axi = await setup(dut)
    before = model_errors(dut)
    rnd = random.Random(7)

    def pauses():
        # long stalls too: the bug window is RREADY still low after the memory transaction has ended
        while True:
            for _ in range(rnd.randrange(1, 20)):
                yield False
            for _ in range(rnd.choice([0, 1, 3, 10, 30, 50])):
                yield True

    for beats in [1, 4, 16, 64]:
        base = 0x9000
        chunks = [pattern(4 * beats, 70 + beats + i) for i in range(6)]
        axi.write_if.w_channel.set_pause_generator(pauses())
        writes = [cocotb.start_soon(axi.write(base + 4 * beats * i, c)) for i, c in enumerate(chunks)]
        for t in writes:
            if not t.done():
                await with_timeout(t, timeout_us=2000)
        axi.write_if.w_channel.clear_pause_generator()
        axi.write_if.w_channel.pause = False
        await wait_idle(dut)
        assert bytes(mem_get(dut, base, 4 * beats * len(chunks))) == b"".join(chunks), f"{beats} beats: write"

        axi.read_if.r_channel.set_pause_generator(pauses())
        reads = [cocotb.start_soon(axi.read(base + 4 * beats * i, 4 * beats)) for i in range(len(chunks))]
        for i, t in enumerate(reads):
            result = t.result() if t.done() else await with_timeout(t, timeout_us=2000)
            assert result.data == chunks[i], f"{beats} beats: read burst {i}"
        axi.read_if.r_channel.clear_pause_generator()
        axi.read_if.r_channel.pause = False
        await wait_idle(dut)
    check_no_model_errors(dut, before)


@cocotb.test()
async def write_waits_for_later_read_test(dut):
    """
    DMA copy with different source / destination alignment (chip-di-owas crypto DMA bypass): the source is read in
    two bursts (15 beats up to a 64 B boundary, then 1 beat), the destination is written in one 16-beat burst whose
    W data comes from the reads. When the first read is done, the AW (with W data) and the second AR are both
    pending; the write can only finish after the second read has returned its data.
    """
    axi = await setup(dut)
    before = model_errors(dut)
    src = 0x0FC4
    dst = 0x3000
    data = pattern(64, 80)
    await with_timeout(axi.write(src, data))
    await wait_idle(dut)

    r_beats = 0
    w_beats = 0
    done = False

    async def count():
        nonlocal r_beats, w_beats
        while not done:
            await RisingEdge(dut.clk)
            if dut.s_axi_rvalid.value == 1 and dut.s_axi_rready.value == 1:
                r_beats += 1
            if dut.s_axi_wvalid.value == 1 and dut.s_axi_wready.value == 1:
                w_beats += 1

    def w_available():
        # a W beat can only be sent once its word has been read (the DMA forwards read data to the write side)
        while True:
            yield w_beats >= r_beats

    mon = cocotb.start_soon(count())
    axi.write_if.w_channel.set_pause_generator(w_available())
    r1 = cocotb.start_soon(axi.read(src, 60))
    r2 = cocotb.start_soon(axi.read(src + 60, 4))
    await RisingEdge(dut.s_axi_rvalid)  # the DMA issues the AW once the first data arrives
    w = cocotb.start_soon(axi.write(dst, data))
    try:
        for t in (r1, r2, w):
            if not t.done():
                await with_timeout(t, timeout_us=200)
    finally:
        done = True
        mon.cancel()
        axi.write_if.w_channel.clear_pause_generator()
        axi.write_if.w_channel.pause = False
    assert r1.result().data + r2.result().data == data
    await wait_idle(dut)
    assert bytes(mem_get(dut, dst, 64)) == data
    check_no_model_errors(dut, before)


@cocotb.test()
async def read_waits_for_write_test(dut):
    """
    Mirror case: a master with a small buffer (8 words) reads a 16-beat burst and holds RREADY low when its buffer
    is full, until it has written the buffered words out in a separate write burst.
    """
    axi = await setup(dut)
    before = model_errors(dut)
    src = 0x2000
    dst = 0x3400
    data = pattern(64, 81)
    await with_timeout(axi.write(src, data))
    await wait_idle(dut)

    r_beats = 0
    w_beats = 0
    done = False

    async def count():
        nonlocal r_beats, w_beats
        while not done:
            await RisingEdge(dut.clk)
            if dut.s_axi_rvalid.value == 1 and dut.s_axi_rready.value == 1:
                r_beats += 1
            if dut.s_axi_wvalid.value == 1 and dut.s_axi_wready.value == 1:
                w_beats += 1

    def r_space():
        while True:
            yield r_beats - w_beats >= 8  # buffer full: RREADY low

    def w_available():
        while True:
            yield w_beats >= r_beats

    mon = cocotb.start_soon(count())
    axi.read_if.r_channel.set_pause_generator(r_space())
    axi.write_if.w_channel.set_pause_generator(w_available())
    r = cocotb.start_soon(axi.read(src, 64))
    await RisingEdge(dut.s_axi_rvalid)
    w1 = cocotb.start_soon(axi.write(dst, data[:32]))
    w2 = cocotb.start_soon(axi.write(dst + 32, data[32:]))
    try:
        for t in (r, w1, w2):
            if not t.done():
                await with_timeout(t, timeout_us=200)
    finally:
        done = True
        mon.cancel()
        for ch in (axi.read_if.r_channel, axi.write_if.w_channel):
            ch.clear_pause_generator()
            ch.pause = False
    assert r.result().data == data
    await wait_idle(dut)
    assert bytes(mem_get(dut, dst, 64)) == data
    check_no_model_errors(dut, before)


@cocotb.test()
async def unsupported_burst_test(dut):
    """
    FIXED / WRAP bursts with more than one beat and narrow bursts with more than one beat are not executed: SLVERR,
    the memory is not accessed (no CE# low), read data is zero. Single beats of any type and size still work, and
    a normal burst running concurrently is not disturbed.
    """
    axi = await setup(dut)
    before = model_errors(dut)
    base = 0x8000
    old = pattern(64, 60)
    await with_timeout(axi.write(base, old))
    await wait_idle(dut)

    ce_falls = 0

    async def count_ce():
        nonlocal ce_falls
        while True:
            await FallingEdge(dut.gen_memory[0].Memory.CE_n)
            ce_falls += 1

    mon = cocotb.start_soon(count_ce())
    cases = [
        ("FIXED", AxiBurstType.FIXED, 2, 16),
        ("WRAP", AxiBurstType.WRAP, 2, 16),
        ("narrow byte", AxiBurstType.INCR, 0, 8),
        ("narrow half word", AxiBurstType.INCR, 1, 8),
        ("narrow FIXED", AxiBurstType.FIXED, 0, 4),
    ]
    for name, burst, size, length in cases:
        result = await with_timeout(axi.write(base, bytes([0xEE] * length), burst=burst, size=size, awid=0x21))
        assert result.resp == AxiResp.SLVERR, f"{name} write: {result.resp}"
        result = await with_timeout(axi.read(base, length, burst=burst, size=size, arid=0x22))
        assert result.resp == AxiResp.SLVERR, f"{name} read: {result.resp}"
        assert result.data == bytes(length), f"{name} read data"
    mon.cancel()
    assert ce_falls == 0, f"unsupported bursts accessed the memory ({ce_falls} CE# low phases)"
    assert bytes(mem_get(dut, base, 64)) == old, "unsupported write changed the memory"

    # single beats are supported with any burst type and size
    result = await with_timeout(axi.write(base + 1, b"\x5a", size=0))
    assert result.resp == AxiResp.OKAY
    result = await with_timeout(axi.write(base + 6, b"\x12\x34", size=1, burst=AxiBurstType.FIXED))
    assert result.resp == AxiResp.OKAY
    result = await with_timeout(axi.read(base + 4, 4, burst=AxiBurstType.FIXED))
    assert result.resp == AxiResp.OKAY
    expected = bytearray(old)
    expected[1] = 0x5A
    expected[6:8] = b"\x12\x34"
    assert result.data == bytes(expected[4:8])

    # an unsupported write running concurrently with normal traffic
    data = pattern(64, 61)
    bad = cocotb.start_soon(axi.write(base, bytes(16), burst=AxiBurstType.WRAP, awid=0x21))
    await with_timeout(axi.write(base + 0x40, data, awid=0x20))
    result = await with_timeout(axi.read(base + 0x40, 64, arid=0x20))
    assert result.resp == AxiResp.OKAY and result.data == data
    bad_result = bad.result() if bad.done() else await with_timeout(bad)
    assert bad_result.resp == AxiResp.SLVERR
    await wait_idle(dut)
    assert bytes(mem_get(dut, base, 64)) == bytes(expected), "concurrent unsupported write changed the memory"
    check_no_model_errors(dut, before)


@cocotb.test()
async def mixed_masters_test(dut):
    """
    CPU style single beat accesses (with byte strobes) running concurrently with DMA style bursts, each with its
    own crossbar ID. The controller serializes them; every transaction must complete with its own data.
    """
    axi = await setup(dut)
    before = model_errors(dut)
    rnd = random.Random(5)
    await with_timeout(axi.write(0x6000, bytes(256)))  # the model memory starts as X
    await wait_idle(dut)

    async def cpu():
        ref = {a: 0 for a in range(0x6000, 0x6100)}
        for i in range(40):
            addr = 0x6000 + 4 * rnd.randrange(64)
            if rnd.random() < 0.5:
                offset, size = rnd.choice([(0, 4), (0, 1), (3, 1), (2, 2)])
                data = bytes(rnd.randrange(256) for _ in range(size))
                await axi.write(addr + offset, data, awid=0x43)
                for j, b in enumerate(data):
                    ref[addr + offset + j] = b
            else:
                result = await axi.read(addr, 4, arid=0x43)
                for j in range(4):
                    if addr + j in ref:
                        assert result.data[j] == ref[addr + j], f"CPU read {addr + j:#x}"
        return ref

    async def dma(axi_id, base):
        for i in range(6):
            data = pattern(64, axi_id * 16 + i)
            await axi.write(base + 64 * i, data, awid=axi_id)
            result = await axi.read(base + 64 * i, 64, arid=axi_id)
            assert result.data == data, f"DMA {axi_id:#x} burst {i}"

    tasks = [cocotb.start_soon(dma(0x20, 0x7000)), cocotb.start_soon(dma(0x00, 0x7800))]
    ref = await with_timeout(cpu(), timeout_us=2000)
    for t in tasks:
        if not t.done():
            await with_timeout(t, timeout_us=2000)
        t.result()  # raises the DMA task's assertion, if any
    await wait_idle(dut)

    result = await with_timeout(axi.read(0x6000, 256))
    for a, b in ref.items():
        assert result.data[a - 0x6000] == b, f"byte {a:#x}"
    await wait_idle(dut)
    check_no_model_errors(dut, before)


async def max_ce_low_time(dut, coro, timeout_us=TIMEOUT_US):
    """Run an AXI transaction and return the longest CE# low time (ns) of chip 0 during it."""
    longest = 0
    done = False

    async def monitor():
        nonlocal longest
        while not done:
            await FallingEdge(dut.gen_memory[0].Memory.CE_n)
            t_fall = get_sim_time("ns")
            await RisingEdge(dut.gen_memory[0].Memory.CE_n)
            longest = max(longest, get_sim_time("ns") - t_fall)

    mon = cocotb.start_soon(monitor())
    result = await with_timeout(coro, timeout_us)
    await wait_idle(dut)
    done = True
    mon.cancel()
    return longest, result


@cocotb.test()
@cocotb.parametrize(t_cem_ns=[8000])  # room temperature only
async def ce_low_time_test(dut, t_cem_ns):
    """
    CE# must not stay low longer than tCEM (8 us standard / 3 us extended temperature), otherwise the device
    cannot refresh. Checked for the longest AXI burst (256 beats) with and without AXI stalls.
    """
    axi = await setup(dut)
    data = pattern(1024, 21)

    t_write, _ = await max_ce_low_time(dut, axi.write(0x0000, data))
    dut._log.info(f"CE# low: write {t_write} ns")
    t_read, result = await max_ce_low_time(dut, axi.read(0x0000, 1024))
    dut._log.info(f"CE# low: read {t_read} ns")
    assert result.data == data

    axi.read_if.r_channel.set_pause_generator(itertools.cycle([0] * 8 + [1] * 200))
    # RREADY only 8 of 208 cycles: every restart delivers about one word, slow but must not hang
    t_read_stall, result = await max_ce_low_time(dut, axi.read(0x0000, 1024), timeout_us=5000)
    assert result.data == data
    axi.read_if.r_channel.clear_pause_generator()
    axi.read_if.r_channel.pause = False  # clear keeps the last pause state

    axi.write_if.w_channel.set_pause_generator(itertools.cycle([0] * 8 + [1] * 200))
    t_write_stall, _ = await max_ce_low_time(dut, axi.write(0x0000, data), timeout_us=5000)
    axi.write_if.w_channel.clear_pause_generator()
    axi.write_if.w_channel.pause = False  # clear keeps the last pause state

    dut._log.info(
        f"CE# low: write {t_write} ns, read {t_read} ns, read with RREADY stall {t_read_stall} ns, "
        f"write with WVALID stall {t_write_stall} ns"
    )
    for name, t in [("write", t_write), ("read", t_read), ("stalled read", t_read_stall),
                    ("stalled write", t_write_stall)]:
        assert t <= t_cem_ns, f"{name}: CE# low for {t} ns > tCEM {t_cem_ns} ns"


@cocotb.test()
@cocotb.parametrize(seed=[1, 2, 3])
async def random_stall_test(dut, seed):
    """
    Bursts of random length and start word with random WVALID / RREADY stalls, mixed with unstalled bursts.
    The data must be correct, the memory protocol clean and CE# never low longer than tCEM.
    """
    axi = await setup(dut)
    before = model_errors(dut)
    rnd = random.Random(seed)

    longest = 0
    done = False

    async def monitor():
        nonlocal longest
        while not done:
            await FallingEdge(dut.gen_memory[0].Memory.CE_n)
            t_fall = get_sim_time("ns")
            await RisingEdge(dut.gen_memory[0].Memory.CE_n)
            longest = max(longest, get_sim_time("ns") - t_fall)

    cocotb.start_soon(monitor())
    reference = {}
    for n in range(12):
        words = rnd.choice([1, 2, 3, 7, 16, 33, 64, 256])
        addr = 0x4000 + 4 * rnd.randrange(0, 257 - words)  # stays inside one 4 KB page
        data = bytes(rnd.randrange(256) for _ in range(4 * words))

        def pauses():
            while True:
                yield rnd.random() < 0.3 if rnd.random() < 0.7 else 0

        if n % 3:
            axi.write_if.w_channel.set_pause_generator(pauses())
        await with_timeout(axi.write(addr, data))
        axi.write_if.w_channel.clear_pause_generator()
        axi.write_if.w_channel.pause = False  # clear keeps the last pause state
        await wait_idle(dut)
        for i, b in enumerate(data):
            reference[addr + i] = b

        if n % 2:
            axi.read_if.r_channel.set_pause_generator(pauses())
        result = await with_timeout(axi.read(addr, 4 * words))
        axi.read_if.r_channel.clear_pause_generator()
        axi.read_if.r_channel.pause = False  # clear keeps the last pause state
        await wait_idle(dut)
        assert result.data == data, f"burst {n}: {words} words at {addr:#x}"

    # everything written so far must still be there (no stray writes from cut bursts)
    lo, hi = min(reference), max(reference) + 1
    result = await with_timeout(axi.read(lo, hi - lo))
    for a in range(lo, hi):
        if a in reference:
            assert result.data[a - lo] == reference[a], f"byte {a:#x}"
    await wait_idle(dut)
    done = True

    dut._log.info(f"longest CE# low {longest} ns")
    assert longest <= 8000
    check_no_model_errors(dut, before)


@cocotb.test()
async def latency_setup_test(dut):
    """
    Four chips share one clock and the controller samples at a fixed time, so the chips must run with fixed
    read latency (MR0[5] = 1) - otherwise a refresh collision in one chip shifts its data (issue #27).
    """
    axi = await setup(dut)
    await with_timeout(axi.read(0x0, 4))
    await wait_idle(dut)
    for i, m in enumerate(models(dut)):
        assert int(m.mr0.value) & 0x20, f"chip {i}: MR0 = {int(m.mr0.value):#04x}, latency is still variable"


# --------------------------------------------------------------------------------------------------------------
def run(test_filter=None, pushout=0, wave=False, init_byte=-1, cem_max_cycles=None, bus_clock_negedge=1, r_fifo_depth=0, build_suffix="",
        clk_period_ns=20):
    """cem_max_cycles=None keeps the controller default (derived from CLK_FREQ_MIN_HZ)."""
    proj_path = Path(__file__).resolve().parent
    sources = [
        proj_path / "../../src/OSPIController_quad.v",
        proj_path / "../../src/OctalSPI_quad.v",
        proj_path / "../../src/AXIInterface.v",
        proj_path / "../../src/OctalMemoryController_quad.v",
        proj_path / "../../test/sim_models/aps12808l_model.v",
        proj_path / "../../test/cocotb_wrapper/Quad_Octal_Memory_Controller_apm_wrapper.v",
    ]
    parameters = {
        "DATA_WIDTH": 32,
        "ADDR_WIDTH": 32,
        "MEM_ADDR_BITS": 14,
        "REFRESH_PUSHOUT_PERCENT": pushout,
        # the controller holds RESET# low for 400 us, which covers tPU of the device
        "T_PU": 0,
        "INIT_BYTE": init_byte,
        "BUS_CLOCK_NEGEDGE": bus_clock_negedge,
        "R_FIFO_DEPTH": r_fifo_depth,
    }
    if cem_max_cycles is not None:
        parameters["CEM_MAX_CYCLES"] = cem_max_cycles
    sim = os.getenv("SIM", "icarus")
    runner = get_runner(sim)
    build_dir = proj_path / f"sim_build_apmemory_{pushout}_{init_byte}_{cem_max_cycles or 'def'}_{bus_clock_negedge}_{r_fifo_depth}{build_suffix}"
    if clk_period_ns != 20:
        build_dir = build_dir.with_name(build_dir.name + f"_{clk_period_ns}ns")
    runner.build(
        sources=sources,
        hdl_toplevel="Quad_Octal_Memory_Controller_apm_wrapper",
        build_args=["-g2012"] if sim == "icarus" else [],
        build_dir=build_dir,
        always=True,
        waves=wave,
        parameters=parameters,
    )
    runner.test(
        hdl_toplevel="Quad_Octal_Memory_Controller_apm_wrapper",
        test_module="test_quad_octal_apmemory_model",
        test_filter=test_filter,
        build_dir=build_dir,
        waves=wave,
        parameters=parameters,
        extra_env={"CLK_PERIOD_NS": str(clk_period_ns)},
    )


def test_quad_octal_apmemory_model():
    run(test_filter=os.getenv("TEST_FILTER"), wave=bool(os.getenv("WAVES")))


def test_quad_octal_apmemory_model_posedge_bus_clock():
    """All tests with the bus clock from the posedge register (BUS_CLOCK_NEGEDGE = 0, edge aligned)."""
    run(test_filter=os.getenv("TEST_FILTER"), bus_clock_negedge=0)


def test_quad_octal_apmemory_model_slow_clock():
    """
    Data and CE# tests at the lowest supported clk (CLK_FREQ_MIN_HZ = 10 MHz, 100 ns): the memory model checks tCEM
    in real time, ce_low_time_test checks the longest CE# low time of a 256-beat burst.
    """
    run(
        test_filter="write_read_roundtrip_test|single_word_consistency_test|back_to_back_test|ce_low_time_test"
        "|random_stall_test|dma_copy_test|queued_bursts_with_stall_test",
        clk_period_ns=100,
    )


def test_quad_octal_apmemory_model_short_cem():
    """Data tests with a very short CE# limit (60 cycles), so nearly every burst is split by the tCEM counter."""
    run(
        test_filter="write_read_roundtrip_test|unaligned_word_test|single_word_consistency_test|byte_strobe_test"
        "|random_stall_test|read_backpressure_test|write_backpressure_test|dma_copy_test|mixed_masters_test"
        "|queued_bursts_with_stall_test|write_waits_for_later_read_test|read_waits_for_write_test",
        cem_max_cycles=60,
    )


def test_quad_octal_apmemory_model_hw_scenario():
    """Run the board scenario, writes the reads to $HW_SCENARIO (skipped if not set)."""
    if os.getenv("HW_SCENARIO"):
        run(test_filter="hw_scenario_test", init_byte=0xEE)


def test_quad_octal_apmemory_model_throughput():
    """Measure cycles per burst, writes a table to $THROUGHPUT (skipped if not set)."""
    if os.getenv("THROUGHPUT"):
        run(test_filter="throughput_test")


def test_quad_octal_apmemory_model_buffer_estimate():
    """
    Read stall patterns with a testbench R FIFO of several depths in front of the controller, to estimate what a
    skid buffer would gain. Appends a table to $BUFFER_ESTIMATE (skipped if not set).
    """
    if os.getenv("BUFFER_ESTIMATE"):
        for depth in [0, 2, 4, 8, 32]:
            os.environ["BUFFER_DEPTH"] = str(depth)
            run(test_filter="buffer_estimate_test", r_fifo_depth=depth)


def test_quad_octal_apmemory_model_sustained():
    """Long run with one workload, see sustained_test (skipped if $SUSTAINED is not set)."""
    if os.getenv("SUSTAINED"):
        run(test_filter="sustained_test", build_suffix="_" + os.environ["SUSTAINED_PATTERN"])


def test_quad_octal_apmemory_model_refresh_pushout():
    """Same tests with refresh collisions in 50 % of the variable latency reads."""
    run(test_filter="write_read_roundtrip_test|single_word_consistency_test", pushout=50)


if __name__ == "__main__":
    run(test_filter=os.getenv("TEST_FILTER"), wave=True)


# --------------------------------------------------------------------------------------------------------------
# Same AXI scenario as on the board (test/hardware_verification/octal_nexys_video/hw_scenario.py)
HW_DIR = Path(__file__).resolve().parent / "../hardware_verification/octal_nexys_video"


@cocotb.test(skip=os.getenv("HW_SCENARIO") is None)
async def hw_scenario_test(dut):
    import sys

    sys.path.insert(0, str(HW_DIR))
    from hw_scenario import SCENARIO

    axi = await setup(dut)
    out = open(os.environ["HW_SCENARIO"], "w")
    for n, (op, addr, arg) in enumerate(SCENARIO):
        if op == "W":
            data = b"".join(w.to_bytes(4, "little") for w in arg)
            task = cocotb.start_soon(axi.write(addr, data))
        else:
            task = cocotb.start_soon(axi.read(addr, 4 * arg))
        timeout = Timer(200, unit="us")
        if await First(task, timeout) is timeout:
            dut._log.warning(f"step {n} ({op} {addr:#x}) timed out - stopping like the board does")
            if op == "R":
                out.write(f"R {addr:08x}\n")
            break
        await wait_idle(dut)
        if op == "R":
            d = task.result().data
            words = [int.from_bytes(d[i : i + 4], "little") for i in range(0, len(d), 4)]
            out.write(f"R {addr:08x} " + " ".join(f"{w:08x}" for w in words) + "\n")
    out.close()


# --------------------------------------------------------------------------------------------------------------
# Throughput measurement (not a pass/fail test)


@cocotb.test(skip=os.getenv("THROUGHPUT") is None)
async def throughput_test(dut):
    """
    Cycles from the first address handshake to the last R beat / B response, measured on the AXI signals.
    "single" = one burst (latency), "8x" = 8 bursts queued back to back (sustained), optionally with 30 %
    random RREADY / WVALID stalls. Results go to the file named by $THROUGHPUT.
    """
    axi = await setup(dut)
    before = model_errors(dut)
    await with_timeout(axi.write(0, bytes(0x8000)))  # the model memory starts as X
    await wait_idle(dut)

    async def measure(op, beats, count, stall, seed):
        rnd = random.Random(seed)
        first = None
        last = None
        done = False

        async def monitor():
            nonlocal first, last
            cycle = 0
            while not done:
                await RisingEdge(dut.clk)
                cycle += 1
                if first is None and (
                    (dut.s_axi_awvalid.value == 1 and dut.s_axi_awready.value == 1)
                    or (dut.s_axi_arvalid.value == 1 and dut.s_axi_arready.value == 1)
                ):
                    first = cycle
                if (dut.s_axi_rvalid.value == 1 and dut.s_axi_rready.value == 1 and dut.s_axi_rlast.value == 1) or (
                    dut.s_axi_bvalid.value == 1 and dut.s_axi_bready.value == 1
                ):
                    last = cycle

        def pauses():
            while True:
                yield rnd.random() < 0.3

        channel = axi.read_if.r_channel if op == "read" else axi.write_if.w_channel
        if stall:
            channel.set_pause_generator(pauses())
        mon = cocotb.start_soon(monitor())
        tasks = []
        for i in range(count):
            addr = 4 * beats * i
            if op == "read":
                tasks.append(cocotb.start_soon(axi.read(addr, 4 * beats)))
            else:
                tasks.append(cocotb.start_soon(axi.write(addr, pattern(4 * beats, i))))
        for t in tasks:
            if not t.done():
                await with_timeout(t, timeout_us=5000)
        done = True
        await RisingEdge(dut.clk)
        mon.cancel()
        if stall:
            channel.clear_pause_generator()
            channel.pause = False
        await wait_idle(dut)
        return last - first + 1

    lines = ["op     beats  bursts  stall  cycles  cycles/burst  words/cycle  MB/s@50MHz"]
    for op in ["read", "write"]:
        for beats in [1, 4, 16, 64, 256]:
            for count, stall in [(1, False), (8, False), (8, True)]:
                cycles = await measure(op, beats, count, stall, beats * 10 + count)
                words = beats * count
                mbs = 4 * words / (cycles * CLK_PERIOD_NS * 1e-9) / 1e6
                lines.append(
                    f"{op:<6} {beats:>5}  {count:>6}  {'30%' if stall else '-':>5}  {cycles:>6}  "
                    f"{cycles / count:>12.1f}  {words / cycles:>11.3f}  {mbs:>10.1f}"
                )
                dut._log.info(lines[-1])
    with open(os.environ["THROUGHPUT"], "w") as f:
        f.write("\n".join(lines) + "\n")
    check_no_model_errors(dut, before)


@cocotb.test(skip=os.getenv("SUSTAINED") is None)
async def sustained_test(dut):
    """
    Long run with one workload ($SUSTAINED_PATTERN) for $SUSTAINED_MS of simulated time. 8 bursts are kept in
    flight all the time; throughput is reported per 10 ms window and in total, appended to $SUSTAINED.
    read256 / write256: 256-beat INCR bursts. copy16: 16-beat reads and writes alternating (DMA copy).
    read16_stall: 16-beat reads with RREADY low in a random 30 % of the cycles. Read data is checked.
    """
    workload = os.environ["SUSTAINED_PATTERN"]
    duration_ns = float(os.getenv("SUSTAINED_MS", "100")) * 1e6
    window_ns = 10e6
    region = 0x8000  # the model has 2^14 words per chip, the bursts wrap inside the first 32 KB
    axi = await setup(dut)
    before = model_errors(dut)
    reference = bytearray(pattern(region, 1))
    await with_timeout(axi.write(0, bytes(reference)), timeout_us=20000)
    await wait_idle(dut)

    beats = 256 if workload.endswith("256") else 16
    size = 4 * beats
    if workload == "read16_stall":
        rnd = random.Random(7)

        def pauses():
            while True:
                yield rnd.random() < 0.3

        axi.read_if.r_channel.set_pause_generator(pauses())

    async def burst(i):
        addr = (i * size) % region
        if workload.startswith("read") or (workload == "copy16" and i % 2 == 0):
            data = (await axi.read(addr, size)).data
            assert data == reference[addr : addr + size], f"read data wrong at {addr:#x}"
        else:
            data = pattern(size, i)
            await axi.write(addr, data)
            reference[addr : addr + size] = data
        return size

    start = get_sim_time("ns")
    window_start, window_bytes, total_bytes = start, 0, 0
    lines = []
    in_flight = []
    i = 0
    while True:
        while len(in_flight) < 8:
            in_flight.append(cocotb.start_soon(burst(i)))
            i += 1
        done = in_flight.pop(0)
        if not done.done():
            await with_timeout(done, timeout_us=5000)
        nbytes = done.result()
        window_bytes += nbytes
        total_bytes += nbytes
        now = get_sim_time("ns")
        if now - window_start >= window_ns:
            mbs = window_bytes / (now - window_start) * 1e3
            lines.append(f"{workload:<13} {(window_start - start) / 1e6:>6.1f}-{(now - start) / 1e6:>6.1f} ms  {mbs:>6.1f} MB/s")
            dut._log.info(lines[-1])
            window_start, window_bytes = now, 0
        if now - start >= duration_ns:
            break
    for t in in_flight:
        if not t.done():
            await with_timeout(t, timeout_us=5000)
        total_bytes += t.result()
    end = get_sim_time("ns")
    lines.append(
        f"{workload:<13} total {(end - start) / 1e6:.1f} ms, {total_bytes / 1e6:.2f} MB, {i} bursts: "
        f"{total_bytes / (end - start) * 1e3:.1f} MB/s"
    )
    dut._log.info(lines[-1])
    with open(os.environ["SUSTAINED"], "a") as f:
        f.write("\n".join(lines) + "\n")
    check_no_model_errors(dut, before)


@cocotb.test(skip=os.getenv("BUFFER_ESTIMATE") is None)
async def buffer_estimate_test(dut):
    """
    Read 1 KB with different RREADY patterns, measured from the first AR handshake to the last R beat.
    The R FIFO depth (testbench, R_FIFO_DEPTH) stands for a skid buffer in the controller.
    """
    axi = await setup(dut)
    depth = int(os.environ["BUFFER_DEPTH"])
    await with_timeout(axi.write(0, bytes(0x2000)))
    await wait_idle(dut)

    def crypto_consumer(block_busy=165):
        # chip-di-owas crypto path: axi_dma_rd output register (2 words) -> aesStreamAdapter gathers 4 words
        # -> AES128 round based core, 161 cycles per block (+ ~4 cycles to scatter the result).
        # Bypass (CTRL.BYPASS): gather 4 words, then scatter them in 4 cycles (block_busy=4).
        skid = 0
        gathered = 0
        busy = 0
        while True:
            if dut.s_axi_rvalid.value == 1 and dut.s_axi_rready.value == 1:
                skid += 1
            if busy > 0:
                busy -= 1
            elif skid > 0:
                skid -= 1
                gathered += 1
                if gathered == 4:
                    gathered = 0
                    busy = block_busy
            yield skid >= 2  # pause = RREADY low

    def short_stalls(rnd):
        # generic: in 10 % of the cycles a stall of 1..3 cycles starts (arbitration, FIFO almost full)
        while True:
            if rnd.random() < 0.1:
                for _ in range(rnd.randint(1, 3)):
                    yield True
            else:
                yield False

    def random30(rnd):
        while True:
            yield rnd.random() < 0.3

    patterns = [
        ("none", 16, None),
        ("crypto DMA (AES 161 cyc/block)", 16, crypto_consumer),
        ("crypto DMA bypass", 16, lambda: crypto_consumer(4)),
        ("short stalls 10 % x 1-3 cyc", 16, lambda: short_stalls(random.Random(3))),
        ("short stalls 10 % x 1-3 cyc", 64, lambda: short_stalls(random.Random(3))),
        ("random 30 % single cycles", 16, lambda: random30(random.Random(4))),
        ("random 30 % single cycles", 64, lambda: random30(random.Random(4))),
    ]
    total = 1024
    lines = []
    for name, burst, gen in patterns:
        axi.read_if.max_burst_len = burst
        first = None
        last = None
        restarts = 0
        done = False

        async def monitor():
            nonlocal first, last, restarts
            cycle = 0
            ai = dut.Controller.AXI_Interface
            while not done:
                await RisingEdge(dut.clk)
                cycle += 1
                if first is None and dut.s_axi_arvalid.value == 1 and dut.s_axi_arready.value == 1:
                    first = cycle
                if dut.s_axi_rvalid.value == 1 and dut.s_axi_rready.value == 1 and dut.s_axi_rlast.value == 1:
                    last = cycle
                if ai.restart_read.value == 1:
                    restarts += 1

        if gen:
            axi.read_if.r_channel.set_pause_generator(gen())
        mon = cocotb.start_soon(monitor())
        result = await with_timeout(axi.read(0, total), timeout_us=20000)
        assert result.data == bytes(total)
        done = True
        await RisingEdge(dut.clk)
        mon.cancel()
        if gen:
            axi.read_if.r_channel.clear_pause_generator()
            axi.read_if.r_channel.pause = False
        await wait_idle(dut)
        cycles = last - first + 1
        mbs = total / (cycles * CLK_PERIOD_NS * 1e-9) / 1e6
        lines.append(f"{depth}|{name}|{burst}|{cycles}|{restarts}|{mbs:.1f}")
        dut._log.info("BUFFER " + lines[-1])

    # write side (the R FIFO does not change it): WVALID pattern of the crypto DMA in bypass, the adapter delivers
    # 4 words, then gathers the next 4 (4 cycles without data)
    def bypass_source():
        while True:
            for _ in range(4):
                yield False
            for _ in range(4):
                yield True

    for name, gen in [("write none", None), ("write crypto DMA bypass", bypass_source)]:
        axi.write_if.max_burst_len = 16
        first = None
        last = None
        restarts = 0
        done = False

        async def wmonitor():
            nonlocal first, last, restarts
            cycle = 0
            ai = dut.Controller.AXI_Interface
            while not done:
                await RisingEdge(dut.clk)
                cycle += 1
                if first is None and dut.s_axi_awvalid.value == 1 and dut.s_axi_awready.value == 1:
                    first = cycle
                if dut.s_axi_bvalid.value == 1 and dut.s_axi_bready.value == 1:
                    last = cycle
                if ai.restart_write.value == 1:
                    restarts += 1

        if gen:
            axi.write_if.w_channel.set_pause_generator(gen())
        mon = cocotb.start_soon(wmonitor())
        await with_timeout(axi.write(0, bytes(total)), timeout_us=20000)
        done = True
        await RisingEdge(dut.clk)
        mon.cancel()
        if gen:
            axi.write_if.w_channel.clear_pause_generator()
            axi.write_if.w_channel.pause = False
        await wait_idle(dut)
        cycles = last - first + 1
        mbs = total / (cycles * CLK_PERIOD_NS * 1e-9) / 1e6
        lines.append(f"{depth}|{name}|16|{cycles}|{restarts}|{mbs:.1f}")
        dut._log.info("BUFFER " + lines[-1])
    with open(os.environ["BUFFER_ESTIMATE"], "a") as f:
        f.write("\n".join(lines) + "\n")
