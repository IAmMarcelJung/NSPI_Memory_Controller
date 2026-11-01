/*
Copyright (c) 2018 Alex Forencich
Copyright (c) 2026 Timo Braungardt (modifications)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// Language: Verilog 2001
/*
AXI Interface module

Simple interface which accepts read and write requests.
Can create backpreasure so the SPI can write the data out.
*/

`timescale 1ns / 1ps
`default_nettype none

module AXIInterface #(
    // Width of data bus in bits
    parameter DATA_WIDTH = 32,
    // Width of address bus in bits
    parameter ADDR_WIDTH = 16,
    // Width of wstrb (width of data bus in words)
    parameter STRB_WIDTH = (DATA_WIDTH / 8),
    // Width of ID signal
    parameter ID_WIDTH = 8,
    // Extra pipeline register on output
    parameter PIPELINE_OUTPUT = 1'b0
) (
    input wire clk,
    input wire rst_neg,

    // Control Interface Pins
    input  wire                  i_write_word_ready,
    input  wire                  i_read_word_ready,
    input  wire                  i_busy,
    output wire                  o_last_word,
    output wire                  o_write_enable,
    output wire                  o_write_data_valid,  // wvalid of the beat on o_write_data
    output wire                  o_start_transaction,
    output wire [ADDR_WIDTH-1:0] o_address,
    output wire [DATA_WIDTH-1:0] o_write_data,
    input  wire [DATA_WIDTH-1:0] i_read_data,

    /* verilator lint_off UNUSEDSIGNAL */
    // AXI Pins
    input wire [ID_WIDTH-1:0] s_axi_awid,  // write address channel
    input wire [ADDR_WIDTH-1:0] s_axi_awaddr,
    input wire [7:0] s_axi_awlen,  // length of the transaction in words
    input wire [2:0] s_axi_awsize,  // number of bytes per transfer (1, 2, 4, 8, 16, 32, 64, 128)
    input wire [1:0] s_axi_awburst,
    input wire s_axi_awlock,
    input wire [3:0] s_axi_awcache,
    input wire [2:0] s_axi_awprot,
    input wire s_axi_awvalid,
    output wire s_axi_awready,

    input  wire [DATA_WIDTH-1:0] s_axi_wdata,   // write data channel
    input  wire [STRB_WIDTH-1:0] s_axi_wstrb,   // data strobe for bitmasking
    input  wire                  s_axi_wlast,
    input  wire                  s_axi_wvalid,
    output wire                  s_axi_wready,

    output wire [ID_WIDTH-1:0] s_axi_bid,     // write response channel
    output wire [         1:0] s_axi_bresp,
    output wire                s_axi_bvalid,
    input  wire                s_axi_bready,

    input  wire [  ID_WIDTH-1:0] s_axi_arid,     // read address channel
    input  wire [ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire [           7:0] s_axi_arlen,
    input  wire [           2:0] s_axi_arsize,
    input  wire [           1:0] s_axi_arburst,
    input  wire                  s_axi_arlock,
    input  wire [           3:0] s_axi_arcache,
    input  wire [           2:0] s_axi_arprot,
    input  wire                  s_axi_arvalid,
    output wire                  s_axi_arready,

    output wire [  ID_WIDTH-1:0] s_axi_rid,     // read data channel
    output wire [DATA_WIDTH-1:0] s_axi_rdata,
    output wire [           1:0] s_axi_rresp,
    output wire                  s_axi_rlast,
    output wire                  s_axi_rvalid,
    input  wire                  s_axi_rready
    /* verilator lint_on UNUSEDSIGNAL */
);

    localparam WORD_WIDTH = STRB_WIDTH;
    localparam WORD_SIZE = DATA_WIDTH / WORD_WIDTH;

    // bus width assertions
    initial begin
        if (WORD_SIZE * STRB_WIDTH != DATA_WIDTH) begin
            $error("Error: AXI data width not evenly divisble (instance %m)");
            $finish;
        end

        if (2 ** $clog2(WORD_WIDTH) != WORD_WIDTH) begin
            $error("Error: AXI word width must be even power of two (instance %m)");
            $finish;
        end
    end

    localparam [1:0] RESP_OKAY = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;
    localparam [2:0] SIZE_FULL = 3'($clog2(STRB_WIDTH));  // AxSIZE of a full width beat

    // The memory is accessed as one linear word stream from the start address. Bursts that need anything else
    // (FIXED or WRAP with more than one beat, narrow beats in a burst, beats wider than the bus) are not executed:
    // the memory is not touched and the burst gets SLVERR.
    function automatic burst_unsupported(input [7:0] len, input [2:0] size, input [1:0] burst);
        burst_unsupported = (size > SIZE_FULL) | ((len != 8'd0) & ((burst != 2'b01) | (size != SIZE_FULL)));
    endfunction

    wire aw_unsupported = burst_unsupported(s_axi_awlen, s_axi_awsize, s_axi_awburst);
    wire ar_unsupported = burst_unsupported(s_axi_arlen, s_axi_arsize, s_axi_arburst);

    // Read FSM
    localparam [1:0] READ_STATE_IDLE = 2'd0;
    localparam [1:0] READ_STATE_BURST = 2'd1;
    localparam [1:0] READ_STATE_ERROR = 2'd2;  // unsupported burst: return all beats with SLVERR

    reg [1:0] read_state_reg;
    reg [1:0] read_state_next;
    reg read_drop_reg, read_drop_next;  // a word could not be taken, ignore the rest until the restart

    // Write FSM
    localparam [1:0] WRITE_STATE_IDLE = 2'd0;
    localparam [1:0] WRITE_STATE_BURST = 2'd1;
    localparam [1:0] WRITE_STATE_RESP = 2'd2;
    localparam [1:0] WRITE_STATE_DISCARD = 2'd3;  // unsupported burst: take the beats without writing them

    reg write_error_reg, write_error_next;  // the burst gets SLVERR

    reg [1:0] write_state_reg;
    reg [1:0] write_state_next;

    reg mem_rd_en;
    reg mem_write_reg;  // direction of the last started memory transaction
    reg error_rd_en;  // load an error beat (zero data) into the output register

    reg [ID_WIDTH-1:0] read_id_reg, read_id_next;
    reg [ADDR_WIDTH-1:0] read_addr_reg, read_addr_next;
    reg [7:0] read_count_reg, read_count_next;
    reg [2:0] read_size_reg, read_size_next;
    reg [1:0] read_burst_reg, read_burst_next;
    reg [ID_WIDTH-1:0] write_id_reg, write_id_next;
    reg [ADDR_WIDTH-1:0] write_addr_reg, write_addr_next;
    reg [7:0] write_count_reg, write_count_next;
    reg [2:0] write_size_reg, write_size_next;
    reg [1:0] write_burst_reg, write_burst_next;

    reg s_axi_awready_reg, s_axi_awready_next;
    reg [ID_WIDTH-1:0] s_axi_bid_reg, s_axi_bid_next;
    reg [1:0] s_axi_bresp_reg, s_axi_bresp_next;
    reg s_axi_bvalid_reg, s_axi_bvalid_next;
    reg s_axi_arready_reg, s_axi_arready_next;
    reg [ID_WIDTH-1:0] s_axi_rid_reg, s_axi_rid_next;
    reg [DATA_WIDTH-1:0] s_axi_rdata_reg;
    reg [1:0] s_axi_rresp_reg, s_axi_rresp_next;
    reg s_axi_rlast_reg, s_axi_rlast_next;
    reg s_axi_rvalid_reg, s_axi_rvalid_next;
    reg [ID_WIDTH-1:0] s_axi_rid_pipe_reg;
    reg [DATA_WIDTH-1:0] s_axi_rdata_pipe_reg;
    reg [1:0] s_axi_rresp_pipe_reg;
    reg s_axi_rvalid_pipe_reg;

    assign s_axi_awready = s_axi_awready_reg;
    // the signal has to be passed through immediately from the controller to the axi interface - otherwise we have a clock cycle delay
    // only during the burst: the controller also requests words after the last beat, these must not take the beats of the next burst
    assign s_axi_wready = (i_write_word_ready & mem_write_reg & (write_state_reg == WRITE_STATE_BURST)) |
                          (write_state_reg == WRITE_STATE_DISCARD);
    assign s_axi_bid = s_axi_bid_reg;
    assign s_axi_bresp = s_axi_bresp_reg;
    assign s_axi_bvalid = s_axi_bvalid_reg;
    assign s_axi_arready = s_axi_arready_reg;
    assign s_axi_rid = PIPELINE_OUTPUT ? s_axi_rid_pipe_reg : s_axi_rid_reg;
    assign s_axi_rdata = PIPELINE_OUTPUT ? s_axi_rdata_pipe_reg : s_axi_rdata_reg;
    assign s_axi_rresp = PIPELINE_OUTPUT ? s_axi_rresp_pipe_reg : s_axi_rresp_reg;
    assign s_axi_rlast = s_axi_rlast_reg;//PIPELINE_OUTPUT ? s_axi_rlast_pipe_reg : s_axi_rlast_reg;
    assign s_axi_rvalid = PIPELINE_OUTPUT ? s_axi_rvalid_pipe_reg : s_axi_rvalid_reg;

    // A burst is split into several memory transactions when the master stalls (WVALID low / RREADY low) or the
    // memory needs CE# high (tCEM). The controller then ends the memory transaction and goes idle while the AXI
    // burst is still running, it is restarted here at the address of the next beat.
    //
    // Read and write bursts are independent: while one burst is stalled and the memory is idle, the other direction
    // may run a memory transaction (new burst or restart), then the stalled burst continues. A master may make its
    // write data depend on a read (DMA copy with different source and destination alignment) or its RREADY depend
    // on a write (small buffer), so neither direction may block the other while it waits for the master.
    //
    // One memory transaction at a time: a start is granted only while the memory is idle and no address ready is
    // raised (the ready is registered, the handshake follows in the next cycle and starts the transaction). If both
    // directions want to start, they take turns.
    wire read_can_load = ~s_axi_rvalid_reg | s_axi_rready;  // output register free in this cycle
    wire aw_handshake = s_axi_awready & s_axi_awvalid;
    wire ar_handshake = s_axi_arready & s_axi_arvalid;
    wire mem_free = ~i_busy & ~s_axi_awready & ~s_axi_arready;

    wire want_restart_write = (write_state_reg == WRITE_STATE_BURST) & s_axi_wvalid;
    // a write starts with its first data beat, so a pending AW without W doesn't take the memory
    wire want_new_write = (write_state_reg == WRITE_STATE_IDLE) & s_axi_awvalid & s_axi_wvalid;
    wire want_restart_read = (read_state_reg == READ_STATE_BURST) & read_can_load;
    wire want_new_read = (read_state_reg == READ_STATE_IDLE) & s_axi_arvalid;
    wire want_write = want_restart_write | want_new_write;
    wire want_read = want_restart_read | want_new_read;
    wire prefer_read = mem_write_reg;  // the last transaction was a write
    wire grant_write = mem_free & want_write & (~want_read | ~prefer_read);
    wire grant_read = mem_free & want_read & (~want_write | prefer_read);

    wire restart_write = grant_write & want_restart_write;
    wire restart_read = grant_read & want_restart_read;
    wire accept_write = grant_write & want_new_write;
    wire accept_read = grant_read & want_new_read;

    wire start_write = (aw_handshake & ~aw_unsupported) | restart_write;
    wire start_read = (ar_handshake & ~ar_unsupported) | restart_read;
    assign o_start_transaction = start_write | start_read;
    // direction of the memory transaction that starts in this cycle or is running
    wire mem_write = start_write | (~start_read & mem_write_reg);
    assign o_write_enable = mem_write;
    assign o_address = mem_write ? write_addr_next : read_addr_next;
    assign o_write_data = s_axi_wdata;
    assign o_write_data_valid = s_axi_wvalid;
    wire read_word_ready = i_read_word_ready & ~mem_write_reg;

    wire read_last_word;
    // on the address handshake rlast still belongs to the previous burst, so use arlen there
    // the controller stops after the beat flagged as last: the real last beat, or the current one if it cannot be
    // taken (output register full) or was already dropped - the rest is read again after a restart
    assign read_last_word = (s_axi_arready & s_axi_arvalid) ? (s_axi_arlen == 0) :
                            restart_read ? (read_count_reg == 0) :
                            (read_count_reg == 0) | read_drop_reg | (read_word_ready & ~read_can_load);
    wire write_last_word;
    assign write_last_word = s_axi_wlast;

    assign o_last_word = mem_write ? write_last_word : read_last_word;

    always @* begin
        write_state_next = WRITE_STATE_IDLE;

        write_id_next = write_id_reg;
        write_addr_next = write_addr_reg;
        write_count_next = write_count_reg;
        write_size_next = write_size_reg;
        write_burst_next = write_burst_reg;
        write_error_next = write_error_reg;

        s_axi_awready_next = 1'b0;
        s_axi_bid_next = s_axi_bid_reg;
        s_axi_bresp_next = s_axi_bresp_reg;
        s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;

        case (write_state_reg)
            WRITE_STATE_IDLE: begin
                s_axi_awready_next = accept_write;

                // reviece address etc. of the transaction
                if (s_axi_awready && s_axi_awvalid) begin
                    write_id_next = s_axi_awid;
                    write_addr_next = s_axi_awaddr;
                    write_count_next = s_axi_awlen;
                    write_size_next = s_axi_awsize < 3'($clog2(STRB_WIDTH)) ? s_axi_awsize :
                        3'($clog2(STRB_WIDTH));
                    write_burst_next = s_axi_awburst;
                    write_error_next = aw_unsupported;

                    s_axi_awready_next = 1'b0;
                    write_state_next = aw_unsupported ? WRITE_STATE_DISCARD : WRITE_STATE_BURST;
                end else begin
                    write_state_next = WRITE_STATE_IDLE;
                end
            end
            WRITE_STATE_BURST: begin
                if (s_axi_wready && s_axi_wvalid) begin
                    if (write_burst_reg != 2'b00) begin
                        write_addr_next = write_addr_reg + (1 << write_size_reg);
                    end
                    write_count_next = write_count_reg - 1;
                    if (write_count_reg > 0) begin
                        write_state_next = WRITE_STATE_BURST;
                    end else begin
                        // the response is sent when the controller has written the data to the memory
                        write_state_next = WRITE_STATE_RESP;
                    end
                end else begin
                    write_state_next = WRITE_STATE_BURST;
                end
            end
            WRITE_STATE_DISCARD: begin
                if (s_axi_wvalid) begin
                    write_count_next = write_count_reg - 1;
                    write_state_next = (write_count_reg > 0) ? WRITE_STATE_DISCARD : WRITE_STATE_RESP;
                end else begin
                    write_state_next = WRITE_STATE_DISCARD;
                end
            end
            WRITE_STATE_RESP: begin
                // the data is in the memory once the write transaction has ended (a read may run already)
                if (~(i_busy & mem_write_reg) && (s_axi_bready || !s_axi_bvalid)) begin
                    s_axi_bid_next = write_id_reg;
                    s_axi_bresp_next = write_error_reg ? RESP_SLVERR : RESP_OKAY;
                    s_axi_bvalid_next = 1'b1;
                    s_axi_awready_next = 1'b0;
                    write_state_next = WRITE_STATE_IDLE;
                end else begin
                    write_state_next = WRITE_STATE_RESP;
                end
            end
        endcase
    end

    always @(posedge clk or negedge rst_neg) begin
        if (!rst_neg) begin
            write_state_reg   <= WRITE_STATE_IDLE;
            write_id_reg      <= {ID_WIDTH{1'b0}};
            write_addr_reg    <= {ADDR_WIDTH{1'b0}};
            write_count_reg   <= 8'd0;
            write_size_reg    <= 3'd0;
            write_burst_reg   <= 2'd0;
            write_error_reg   <= 1'b0;

            s_axi_awready_reg <= 1'b0;
            s_axi_bvalid_reg  <= 1'b0;
            s_axi_bid_reg     <= {ID_WIDTH{1'b0}};
            s_axi_bresp_reg   <= RESP_OKAY;
        end else begin
            write_state_reg <= write_state_next;

            write_id_reg <= write_id_next;
            write_addr_reg <= write_addr_next;
            write_count_reg <= write_count_next;
            write_size_reg <= write_size_next;
            write_burst_reg <= write_burst_next;
            write_error_reg <= write_error_next;

            s_axi_awready_reg <= s_axi_awready_next;
            s_axi_bid_reg <= s_axi_bid_next;
            s_axi_bresp_reg <= s_axi_bresp_next;
            s_axi_bvalid_reg <= s_axi_bvalid_next;
        end
    end

    always @* begin
        read_state_next = READ_STATE_IDLE;

        mem_rd_en = 1'b0;
        error_rd_en = 1'b0;

        s_axi_rid_next = s_axi_rid_reg;
        s_axi_rresp_next = s_axi_rresp_reg;
        s_axi_rlast_next = s_axi_rlast_reg;
        s_axi_rvalid_next = s_axi_rvalid_reg && !(s_axi_rready || (PIPELINE_OUTPUT && !s_axi_rvalid_pipe_reg));

        read_id_next = read_id_reg;
        read_addr_next = read_addr_reg;
        read_count_next = read_count_reg;
        read_size_next = read_size_reg;
        read_burst_next = read_burst_reg;
        read_drop_next = read_drop_reg & ~restart_read;

        s_axi_arready_next = 1'b0;

        case (read_state_reg)
            READ_STATE_IDLE: begin
                s_axi_arready_next = accept_read;

                if (s_axi_arready && s_axi_arvalid) begin
                    read_id_next = s_axi_arid;
                    read_addr_next = s_axi_araddr;
                    read_count_next = s_axi_arlen;
                    read_size_next = s_axi_arsize < 3'($clog2(STRB_WIDTH)) ? s_axi_arsize :
                        3'($clog2(STRB_WIDTH));
                    read_burst_next = s_axi_arburst;
                    // rlast is not cleared here: the last beat of the previous burst may still wait in the output
                    // register (RREADY low), rlast is set with every beat that is loaded

                    s_axi_arready_next = 1'b0;
                    read_state_next = ar_unsupported ? READ_STATE_ERROR : READ_STATE_BURST;
                end else begin
                    read_state_next = READ_STATE_IDLE;
                end
            end
            READ_STATE_BURST: begin
                if (read_word_ready & ~read_drop_reg & ~read_can_load) begin
                    // master stalls: drop this word and the rest of the memory transaction, read again after a restart
                    read_drop_next = 1'b1;
                    read_state_next = READ_STATE_BURST;
                end else if (read_word_ready & ~read_drop_reg) begin
                    mem_rd_en = 1'b1;
                    s_axi_rvalid_next = 1'b1;
                    s_axi_rlast_next = (read_count_reg == 0);
                    s_axi_rid_next = read_id_reg;
                    s_axi_rresp_next = RESP_OKAY;
                    if (read_burst_reg != 2'b00) begin
                        read_addr_next = read_addr_reg + (1 << read_size_reg);
                    end
                    read_count_next = read_count_reg - 1;
                    if (read_count_reg > 0) begin
                        read_state_next = READ_STATE_BURST;
                    end else begin
                        s_axi_arready_next = 1'b0;  // the SPI is still busy, accept in IDLE
                        read_state_next = READ_STATE_IDLE;
                    end
                end else begin
                    read_state_next = READ_STATE_BURST;
                end
            end
            READ_STATE_ERROR: begin
                if (read_can_load) begin
                    error_rd_en = 1'b1;
                    s_axi_rvalid_next = 1'b1;
                    s_axi_rlast_next = (read_count_reg == 0);
                    s_axi_rid_next = read_id_reg;
                    s_axi_rresp_next = RESP_SLVERR;
                    read_count_next = read_count_reg - 1;
                    read_state_next = (read_count_reg > 0) ? READ_STATE_ERROR : READ_STATE_IDLE;
                end else begin
                    read_state_next = READ_STATE_ERROR;
                end
            end
            default: begin
                read_state_next = READ_STATE_IDLE;
            end
        endcase
    end

    always @(posedge clk or negedge rst_neg) begin
        if (!rst_neg) mem_write_reg <= 1'b0;
        else if (start_write | start_read) mem_write_reg <= start_write;
    end

    always @(posedge clk or negedge rst_neg) begin
        if (!rst_neg) begin
            read_state_reg        <= READ_STATE_IDLE;
            read_id_reg           <= {ID_WIDTH{1'b0}};
            read_addr_reg         <= {ADDR_WIDTH{1'b0}};
            read_count_reg        <= 8'd0;
            read_size_reg         <= 3'd0;
            read_burst_reg        <= 2'd0;
            read_drop_reg         <= 1'b0;

            s_axi_arready_reg     <= 1'b0;
            s_axi_rvalid_reg      <= 1'b0;
            s_axi_rvalid_pipe_reg <= 1'b0;
            s_axi_rlast_reg       <= 1'b0;
            s_axi_rid_reg         <= {ID_WIDTH{1'b0}};
            s_axi_rresp_reg       <= RESP_OKAY;
            s_axi_rresp_pipe_reg  <= RESP_OKAY;
            s_axi_rid_pipe_reg    <= {ID_WIDTH{1'b0}};
            s_axi_rdata_reg       <= {DATA_WIDTH{1'b0}};
            s_axi_rdata_pipe_reg  <= {DATA_WIDTH{1'b0}};
        end else begin
            read_state_reg <= read_state_next;

            read_id_reg <= read_id_next;
            read_addr_reg <= read_addr_next;
            read_count_reg <= read_count_next;
            read_size_reg <= read_size_next;
            read_burst_reg <= read_burst_next;
            read_drop_reg <= read_drop_next;

            s_axi_arready_reg <= s_axi_arready_next;
            s_axi_rid_reg <= s_axi_rid_next;
            s_axi_rresp_reg <= s_axi_rresp_next;
            s_axi_rlast_reg <= s_axi_rlast_next;
            s_axi_rvalid_reg <= s_axi_rvalid_next;

            if (mem_rd_en) begin
                s_axi_rdata_reg <= i_read_data;
            end else if (error_rd_en) begin
                s_axi_rdata_reg <= {DATA_WIDTH{1'b0}};
            end

            if (!s_axi_rvalid_pipe_reg || s_axi_rready) begin
                s_axi_rid_pipe_reg <= s_axi_rid_reg;
                s_axi_rdata_pipe_reg <= s_axi_rdata_reg;
                s_axi_rresp_pipe_reg <= s_axi_rresp_reg;
                s_axi_rvalid_pipe_reg <= s_axi_rvalid_reg;
            end
        end
    end


endmodule

// AXIInterface sets `default_nettype none, restore the default for the files read after this one
`default_nettype wire
