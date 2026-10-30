`timescale 1ps / 1ps
/*
Behavioral model of the APMemory APS12808L-3OBMx Octal DDR PSRAM (Xccela, datasheet Rev. 1.5).

Written for the cocotb tests of the OWAS quad controller (CHIP_TYPE = 2), because the vendor does not
provide a simulation model. It is cycle based and runs in Icarus Verilog.

Modelled:
 - commands 00h/80h (wrap per MR8), 20h/A0h (linear burst, 1K wrap), 40h/C0h (mode registers), FFh (global reset)
 - instruction on clk 1, address A3/A2/A1/A0 on the edges of clk 2 and 3
 - first data on the rising edge of clk (3 + latency), latency counted from the rising edge of clk 3
 - read latency LC (variable, optional random refresh pushout up to 2*LC) or 2*LC (fixed, MR0[5] = 1)
 - write latency WLC (MR4), register write latency 1, write data masking via DQS/DM (1 = masked)
 - read data driven tDQSCK after the clock edges, DQS toggling with the data

Unknown instructions are only counted in unknown_cmd_count.
Protocol checks (counted in error_count, each with a $display):
 - access before tPU / during reset / before tRST after reset
 - tCPH (CE# high time), tRC (CE# low to CE# low), tCEM (max CE# low time)
 - write with less than 2 bytes, burst not starting on an even address
 - address outside the modelled range (ADDR_BITS), read/write latency fields set to reserved values

Not modelled: setup/hold timing of DQ (the testbench drives zero delay), RBX, PASR, deep power down, half sleep.
*/

module aps12808l_model #(
    parameter integer ADDR_BITS = 16,  // modelled memory size in bytes = 2**ADDR_BITS (the real chip has 24 bits)
    parameter integer REFRESH_PUSHOUT_PERCENT = 0,  // chance that a variable latency read gets 2*LC
    parameter integer SEED = 1,
    parameter integer INIT_BYTE = -1,  // initial memory content, -1 = x
    parameter integer T_DQSCK = 4000,  // ps, 2 ns .. 5.5 ns
    parameter integer T_PU = 150_000_000,  // ps, 150 us device initialization after power up
    parameter integer T_RP = 1_000_000,  // ps, 1 us RESET# low pulse width
    parameter integer T_RST = 2_000_000,  // ps, 2 us reset to command valid
    parameter integer T_CPH = 18_000,  // ps
    parameter integer T_RC = 60_000,  // ps
    parameter integer T_CEM = 8_000_000  // ps, standard temperature range
) (
    input       CE_n,
    input       CLK,
    input       RESET_n,
    inout [7:0] DQ,
    inout       DQS
);

    localparam [7:0] CMD_READ = 8'h00;
    localparam [7:0] CMD_WRITE = 8'h80;
    localparam [7:0] CMD_READ_LINEAR = 8'h20;
    localparam [7:0] CMD_WRITE_LINEAR = 8'hA0;
    localparam [7:0] CMD_MR_READ = 8'h40;
    localparam [7:0] CMD_MR_WRITE = 8'hC0;
    localparam [7:0] CMD_GLOBAL_RESET = 8'hFF;

    reg [7:0] mem[0:(1<<ADDR_BITS)-1];

    // mode registers
    reg [7:0] mr0, mr4, mr8;
    wire [7:0] mr1 = 8'h0D;  // vendor id APM
    wire [7:0] mr2 = 8'h00;
    wire [7:0] mr3 = 8'h40;  // 3V device

    integer error_count;
    integer read_count;
    integer write_count;
    integer mr_write_count;
    integer unknown_cmd_count;
    integer last_latency;  // latency (clocks) of the last read, for the testbench

    // transaction state
    reg [7:0] inst;
    reg [31:0] addr;
    reg [31:0] cur_addr;
    reg [7:0] mr_addr;
    integer rise_cnt;
    integer latency;
    integer byte_cnt;
    integer burst_len;
    integer burst_start;
    reg hybrid_first_pass;
    reg in_transaction;
    reg is_read, is_write, is_mr, is_linear;

    // output drivers
    reg [7:0] dq_out;
    reg dq_oe;
    reg dqs_out;
    reg dqs_oe;
    assign DQ  = dq_oe ? dq_out : 8'bz;
    assign DQS = dqs_oe ? dqs_out : 1'bz;

    // timing bookkeeping
    time t_reset_rise;
    time t_reset_fall;
    time t_ce_fall;
    time t_ce_rise;
    integer seed;
    integer init_i;

    initial begin
        mr0 = 8'h09;  // variable latency, LC = 5, 1/4 drive strength
        mr4 = 8'h40;  // WLC = 5, fast refresh, full array
        mr8 = 8'h05;  // 32 byte hybrid wrap
        error_count = 0;
        read_count = 0;
        write_count = 0;
        mr_write_count = 0;
        unknown_cmd_count = 0;
        last_latency = 0;
        in_transaction = 0;
        dq_oe = 0;
        dqs_oe = 0;
        dq_out = 0;
        dqs_out = 0;
        t_reset_rise = 0;
        t_reset_fall = 0;
        t_ce_fall = 0;
        t_ce_rise = 0;
        seed = SEED;
        if (INIT_BYTE >= 0) for (init_i = 0; init_i < (1 << ADDR_BITS); init_i = init_i + 1) mem[init_i] = INIT_BYTE;
    end

    task automatic error(input [8*96-1:0] msg);
        begin
            error_count = error_count + 1;
            $display("[%0t ps] APS12808L %m ERROR: %0s", $time, msg);
        end
    endtask

    function integer read_lc(input [7:0] mr);
        case (mr[4:2])
            3'b000:  read_lc = 3;
            3'b001:  read_lc = 4;
            3'b010:  read_lc = 5;
            default: read_lc = -1;
        endcase
    endfunction

    function integer write_lc(input [7:0] mr);
        case (mr[7:5])
            3'b000:  write_lc = 3;
            3'b100:  write_lc = 4;
            3'b010:  write_lc = 5;
            default: write_lc = -1;
        endcase
    endfunction

    function integer wrap_len(input [7:0] mr);
        case (mr[1:0])
            2'b00: wrap_len = 16;
            2'b01: wrap_len = 32;
            2'b10: wrap_len = 64;
            2'b11: wrap_len = 1024;
        endcase
    endfunction

    // address of the next byte inside the burst
    task automatic advance_address;
        begin
            if (is_linear || is_mr) begin
                cur_addr[9:0] = cur_addr[9:0] + 1;  // linear burst: 1K wrap (no RBX)
            end else begin
                if ((cur_addr % burst_len) == burst_len - 1)
                    cur_addr = cur_addr - (burst_len - 1);
                else cur_addr = cur_addr + 1;

                // hybrid wrap: after one wrapped pass continue linear in the 1K row
                if (hybrid_first_pass && cur_addr == burst_start) begin
                    hybrid_first_pass = 0;
                    cur_addr = (burst_start - (burst_start % burst_len)) + burst_len;
                    cur_addr[31:10] = burst_start[31:10];
                    burst_len = 1024;
                end
            end
        end
    endtask

    function [7:0] mr_value(input [7:0] ma);
        case (ma)
            8'h00:   mr_value = mr0;
            8'h01:   mr_value = mr1;
            8'h02:   mr_value = mr2;
            8'h03:   mr_value = mr3;
            8'h04:   mr_value = mr4;
            8'h08:   mr_value = mr8;
            default: mr_value = 8'h00;
        endcase
    endfunction

    task automatic write_byte(input [7:0] data, input mask);
        begin
            if (is_mr) begin
                if (byte_cnt == 0 && !mask) begin
                    case (mr_addr)
                        8'h00: begin
                            mr0 = data;
                            if (data[7:6] != 0) error("MR0[7:6] must be written to 0");
                        end
                        8'h04: begin
                            mr4 = data;
                            if (data[4] != 0) error("MR4[4] must be written to 0");
                        end
                        8'h08: begin
                            mr8 = data;
                            if (data[7] != 0) error("MR8[7] must be written to 0");
                        end
                        default: error("mode register write to a read only / unknown register");
                    endcase
                    mr_write_count = mr_write_count + 1;
                    $display("[%0t ps] APS12808L %m: MR%0d <= %02h (clock %0d)", $time, mr_addr, data, rise_cnt);
                end
            end else begin
                if (cur_addr[23:0] >= (1 << ADDR_BITS))
                    error("write address outside of the modelled range");
                else if (!mask) mem[cur_addr[ADDR_BITS-1:0]] = data;
                advance_address;
            end
            byte_cnt = byte_cnt + 1;
        end
    endtask

    task automatic drive_read_byte(input edge_level);
        reg [7:0] data;
        begin
            if (is_mr) data = mr_value(mr_addr);
            else begin
                if (cur_addr[23:0] >= (1 << ADDR_BITS)) begin
                    error("read address outside of the modelled range");
                    data = 8'hxx;
                end else data = mem[cur_addr[ADDR_BITS-1:0]];
                advance_address;
            end
            byte_cnt = byte_cnt + 1;
            dq_oe   <= #(T_DQSCK) 1'b1;
            dq_out  <= #(T_DQSCK) data;
            dqs_out <= #(T_DQSCK) edge_level;
        end
    endtask

    // ---------------- reset ----------------
    always @(negedge RESET_n) t_reset_fall = $time;

    always @(posedge RESET_n) begin
        if ($time > 0 && ($time - t_reset_fall) < T_RP) error("tRP: RESET# low pulse too short");
        t_reset_rise = $time;
        mr0 = 8'h09;
        mr4 = 8'h40;
        mr8 = 8'h05;
    end

    // ---------------- chip enable ----------------
    always @(negedge CE_n) begin
        if ($time < T_PU) error("access before tPU (150 us device initialization)");
        if (RESET_n !== 1'b1) error("access while RESET# is low");
        else if (t_reset_rise > 0 && ($time - t_reset_rise) < T_RST)
            error("tRST: access too early after RESET#");
        if (t_ce_rise > 0 && ($time - t_ce_rise) < T_CPH) error("tCPH: CE# high time too short");
        if (t_ce_fall > 0 && ($time - t_ce_fall) < T_RC) error("tRC: CE# cycle time too short");

        t_ce_fall = $time;
        in_transaction = 1;
        rise_cnt = 0;
        byte_cnt = 0;
        is_read = 0;
        is_write = 0;
        is_mr = 0;
        is_linear = 0;
        latency = 0;
    end

    always @(posedge CE_n) begin
        if (in_transaction) begin
            if (($time - t_ce_fall) > T_CEM) error("tCEM: CE# low too long");
            if (is_write && !is_mr && byte_cnt < 2)
                error("write with less than 2 bytes (minimum burst of the device)");
            if (is_write && !is_mr && byte_cnt % 2 != 0)
                error("write ended in the middle of a clock (odd number of data edges)");
        end
        t_ce_rise = $time;
        in_transaction = 0;
        dq_oe <= #(1000) 1'b0;  // tHZ
        dqs_oe <= #(1000) 1'b0;
    end

    // ---------------- clock ----------------
    always @(posedge CLK) begin
        if (!CE_n && in_transaction) begin
            rise_cnt = rise_cnt + 1;

            if (rise_cnt == 1) begin
                inst = DQ;
                case (inst)
                    CMD_READ, CMD_READ_LINEAR: is_read = 1;
                    CMD_WRITE, CMD_WRITE_LINEAR: is_write = 1;
                    CMD_MR_READ: begin
                        is_read = 1;
                        is_mr   = 1;
                    end
                    CMD_MR_WRITE: begin
                        is_write = 1;
                        is_mr = 1;
                    end
                    CMD_GLOBAL_RESET: ;
                    default: begin
                        unknown_cmd_count = unknown_cmd_count + 1;
                        // counted separately - the real device most likely ignores it
                        $display("[%0t ps] APS12808L %m WARNING: unknown instruction %02h", $time, inst);
                    end
                endcase
                is_linear = (inst == CMD_READ_LINEAR || inst == CMD_WRITE_LINEAR);
            end else if (rise_cnt == 2) begin
                addr[31:24] = DQ;
            end else if (rise_cnt == 3) begin
                addr[15:8] = DQ;
                // the device drives DQS low from the next rising edge after A1 (tCQLZ)
            end else if (rise_cnt == 4 && is_read) begin
                dqs_oe  <= #(T_DQSCK) 1'b1;
                dqs_out <= #(T_DQSCK) 1'b0;
            end

            if (rise_cnt > 3 && (is_read || is_write) && rise_cnt >= 3 + latency) begin
                if (is_write) write_byte(DQ, DQS);
                else drive_read_byte(1'b1);
            end
        end
    end

    always @(negedge CLK) begin
        if (!CE_n && in_transaction) begin
            if (rise_cnt == 1) begin
                if (DQ !== inst && inst !== CMD_GLOBAL_RESET)
                    error("instruction not stable on both edges of the first clock");
            end else if (rise_cnt == 2) begin
                addr[23:16] = DQ;
            end else if (rise_cnt == 3) begin
                addr[7:0] = DQ;
                mr_addr = DQ;
                cur_addr = addr;
                burst_start = addr;
                burst_len = wrap_len(mr8);
                hybrid_first_pass = mr8[2] && (mr8[1:0] != 2'b11);

                if (inst == CMD_GLOBAL_RESET) begin
                    mr0 = 8'h09;
                    mr4 = 8'h40;
                    mr8 = 8'h05;
                end

                if (!is_mr && addr[0] != 0) error("burst does not start on an even address");
                if (!is_mr && addr[31:24] != 0) error("reserved address bits A3 are not 0");

                if (is_write) begin
                    latency = is_mr ? 1 : write_lc(mr4);
                    if (latency < 0) error("reserved write latency code");
                    write_count = write_count + !is_mr;
                end else if (is_read) begin
                    latency = read_lc(mr0);
                    if (latency < 0) error("reserved read latency code");
                    if (!is_mr) begin
                        if (mr0[5]) latency = 2 * latency;  // fixed latency
                        else if (($random(seed) % 100 + 100) % 100 < REFRESH_PUSHOUT_PERCENT)
                            latency = 2 * latency;  // refresh collision
                    end
                    last_latency = latency;
                    read_count = read_count + 1;
                end
            end

            if (rise_cnt > 3 && (is_read || is_write) && rise_cnt >= 3 + latency) begin
                if (is_write) write_byte(DQ, DQS);
                else drive_read_byte(1'b0);
            end
        end
    end

endmodule
