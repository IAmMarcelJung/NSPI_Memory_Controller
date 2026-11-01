`timescale 1ns / 100ps
/*
SPI Controller

Module to handle the SPI-Flash interface via the SPI transmitter.
What it does:
 - prepare the correct opcode for the transmitter based on axi read/write
 - send a write enable before sending a write command
 - multiplex configuration bits instead of data to the transmitter
 - multiplex the bytes to the transmitter based on the burst length
 - counts how many bytes are transfered
*/

module OSPIController_quad #(
    parameter ADDRESS_LENGTH = 32,
    parameter DATA_WIDTH = 32,
    parameter STRB_WIDTH = DATA_WIDTH / 8,
    parameter CHIP_TYPE = -1,  // 1 is Infineon, 2 is APMemory
    // clk frequency: the time based limits below are derived from it (verified at 50 MHz)
    parameter integer CLK_FREQ_HZ = 50_000_000,
    // RESET# of the memories is held low this many clk cycles after reset (default 20 ms).
    // It must cover tRP (1 us) and tPU (150 us after the memory supply is stable) - on the Nexys Video the FMC
    // supply only ramps up after the FPGA configuration, 400 us were too short there.
    parameter integer MEMORY_RESET_CYCLES = (CLK_FREQ_HZ + 49) / 50,
    // Longest memory transaction in clk cycles. CE# must not stay low longer than tCEM (8 us, standard temperature),
    // otherwise the memory cannot refresh. A longer AXI burst is split, the default is 7 us (rounded down).
    parameter integer CEM_MAX_CYCLES = CLK_FREQ_HZ / 1000 * 7 / 1000,
    parameter BUS_CLOCK_NEGEDGE = 1  // see OctalSPI_quad
) (
    input clk,
    input reset_neg,
    input go,

    /* verilator lint_off UNUSEDSIGNAL */
    // because we have 4 chips in parallel, we discard the lowest 2 bits and always write/read and use byte masking
    input  [ADDRESS_LENGTH-1:0] i_address,
    /* verilator lint_on UNUSEDSIGNAL */
    input                       i_write_enable,
    input                       i_write_data_valid,  // wvalid of i_data_write
    input                       i_last_word,
    input  [    STRB_WIDTH-1:0] i_data_word_mask,
    input  [    DATA_WIDTH-1:0] i_data_write,
    output [    DATA_WIDTH-1:0] o_data_read,
    output                      o_busy,
    output                      o_next_word,
    output                      o_recieved_next_word,

    // SPI Pins
    output o_bus_clock,
    output o_bus_clock_neg,
    output [3:0] o_chip_select_neg,
    output o_reset_neg,

    input  [3:0] i_data_strobe,
    output [3:0] o_data_strobe,
    output [3:0] o_data_strobe_output_enable, // 0 input, 1 output

    input [DATA_WIDTH-1:0] i_data,
    output [DATA_WIDTH-1:0] o_data,
    output [3:0] o_data_output_enable  // 0 input, 1 output
);

    // constants
    localparam integer OPCODE_LENGTH = 8;
    localparam DELAY_CYCLES = 10;

    // Chip specific hardcoded constants
    localparam CONFIG_ADDRESS = (CHIP_TYPE == 1) ? 32'h00800002 : 32'h00000000;  // APMemory: MR0
    localparam CONFIG_QSPI_ENABLE = 8'b00000010;
    // APMemory MR0: fixed read latency (MR0[5]), latency code 5 (MR0[4:2] = 010), 1/4 drive strength (MR0[1:0] = 01).
    // The four chips share one clock and are sampled at a fixed time, so a refresh pushout of the variable latency
    // in one chip would shift its data (issue #27).
    localparam [7:0] APMEMORY_MR0 = 8'b0010_1001;
    localparam [DATA_WIDTH-1:0] CONFIG_DATA = (CHIP_TYPE == 1) ? {24'd0, CONFIG_QSPI_ENABLE} : {STRB_WIDTH{APMEMORY_MR0}};

    // After RESET# went high tRST (2 us) has to pass before the first command, RESET_RECOVERY_CYCLES gives 4 us
    // (rounded up).
    localparam integer RESET_CYCLES = MEMORY_RESET_CYCLES;
    localparam integer RESET_RECOVERY_CYCLES = (CLK_FREQ_HZ + 249_999) / 250_000;

    // Logic stuff
    reg  [ OPCODE_LENGTH-1:0] opcode_nxt;
    reg  [ OPCODE_LENGTH-1:0] opcode_reg;
    reg  [ADDRESS_LENGTH-1:0] address_nxt;
    reg  [ADDRESS_LENGTH-1:0] address_reg;
    reg  [    DATA_WIDTH-1:0] config_data_nxt;
    reg  [    DATA_WIDTH-1:0] config_data_reg;
    reg  [    DATA_WIDTH-1:0] data_in_muxed_nxt;
    reg  [    DATA_WIDTH-1:0] data_in_muxed_reg;
    reg  [    STRB_WIDTH-1:0] data_mask_nxt;
    reg  [    STRB_WIDTH-1:0] data_mask_reg;
    reg  [    DATA_WIDTH-1:0] data_read_reg;
    wire                      start_transmission;
    wire                      transmitter_finish;
    wire                      spi_write_next_byte;
    wire                      spi_read_next_byte;
    reg                       write_next_word_nxt;
    reg                       write_next_word_reg;
    reg                       read_next_word_nxt;
    reg                       read_next_word_reg;
    reg                       last_word_nxt;
    reg                       last_word_reg;
    wire [    DATA_WIDTH-1:0] read_word;

    // Config stuff - ToDo: this should be later configured using a second port (issue #16)
    wire                      config_write_address;
    wire                      config_write_data;
    wire                      config_read_data;
    reg                       config_is_config_operation;

    // states control FSM
    localparam NUM_STATES = 6;
    localparam INDEX_STATES_MSB = $clog2(NUM_STATES) - 1;
    localparam [INDEX_STATES_MSB:0] IDLE = 0;
    localparam [INDEX_STATES_MSB:0] READ = 1;
    localparam [INDEX_STATES_MSB:0] WRITE_ENABLE = 2;
    localparam [INDEX_STATES_MSB:0] WRITE = 3;
    localparam [INDEX_STATES_MSB:0] WRITE_CONFIG = 4;
    localparam [INDEX_STATES_MSB:0] WAIT = 5;

    (* dont_touch = "true" *)
    reg [INDEX_STATES_MSB:0] control_state_reg;
    reg [INDEX_STATES_MSB:0] control_state_nxt;

    localparam [OPCODE_LENGTH -1 : 0] OPCODE_READ_INFINEON = 8'hEE;  // Infineon
    localparam [OPCODE_LENGTH -1 : 0] OPCODE_READ_APMEMORY = 8'h20;    // AP Memory    ToDo: cleanup (issue #12)
    localparam [OPCODE_LENGTH -1 : 0] OPCODE_READ = (CHIP_TYPE == 1) ? OPCODE_READ_INFINEON : OPCODE_READ_APMEMORY;
    localparam [OPCODE_LENGTH -1 : 0] OPCODE_WRITE_ENABLE = 8'h06;
    localparam [OPCODE_LENGTH -1 : 0] OPCODE_WRITE_INFINEON = 8'hDE;  // Infineon
    localparam [OPCODE_LENGTH -1 : 0] OPCODE_WRITE_APMEMORY = 8'hA0;  // AP Memory
    localparam [OPCODE_LENGTH -1 : 0] OPCODE_WRITE = (CHIP_TYPE == 1) ? OPCODE_WRITE_INFINEON : OPCODE_WRITE_APMEMORY;
    localparam [OPCODE_LENGTH -1 : 0] OPCODE_WRITE_ANY_REG = (CHIP_TYPE == 1) ? 8'h00 : 8'hC0;  // APMemory: mode register write
    // APMemory has no write enable command
    localparam HAS_WRITE_ENABLE = (CHIP_TYPE == 1);
    //localparam [OPCODE_LENGTH -1 : 0] OPCODE_LONG_ADDRESS_ENABLE = 8'h00;

    // CE# low time of the current memory transaction (tCEM)
    integer cem_counter;
    wire cem_timeout = (cem_counter >= CEM_MAX_CYCLES);
    // the write was cut (master stalled or tCEM): take no more beats, the AXI interface restarts the rest
    reg write_stop_reg;
    reg write_stop_nxt;
    wire write_word_valid = i_write_data_valid & ~write_stop_reg & ~cem_timeout;

    assign o_next_word = (write_next_word_reg | spi_write_next_byte) & ~write_stop_reg & ~cem_timeout;
    assign o_recieved_next_word = read_next_word_reg;

    // Reset and setup sequence of the memories:
    // RESET# low for RESET_CYCLES, wait RESET_RECOVERY_CYCLES, write the configuration (APMemory only), then accept
    // transactions. o_busy is held until the sequence is done.
    reg reset_spi;
    assign o_reset_neg = reset_spi;
    integer reset_counter;
    reg memory_ready;
    reg init_done;
    wire config_start = memory_ready & ~init_done & ~config_is_config_operation & (control_state_reg == IDLE);

    always @(posedge clk or negedge reset_neg) begin
        if (!reset_neg) begin
            reset_counter <= 0;
            reset_spi <= 1'b0;
            memory_ready <= 1'b0;
        end else begin
            if (reset_counter != RESET_CYCLES + RESET_RECOVERY_CYCLES) reset_counter <= reset_counter + 1;
            reset_spi <= (reset_counter >= RESET_CYCLES);
            memory_ready <= (reset_counter == RESET_CYCLES + RESET_RECOVERY_CYCLES);
        end
    end


    OctalSPI_quad #(
        .ADDRESS_LENGTH(ADDRESS_LENGTH),
        .DATA_WIDTH(DATA_WIDTH),
        .CHIP_TYPE(CHIP_TYPE),
        .BUS_CLOCK_NEGEDGE(BUS_CLOCK_NEGEDGE)
    ) SPI_Transmitter (
        .clk(clk),
        .reset_neg(reset_neg),
        .start_transmission(start_transmission),

        .i_address(address_reg),
        .i_opcode(opcode_reg),
        .i_config_read_data(config_read_data),
        .i_config_write_data(config_write_data),
        .i_config_write_address(config_write_address),
        .i_register_write(control_state_reg == WRITE_CONFIG && CHIP_TYPE == 2),
        .i_last_word(last_word_reg),
        .i_data_word_mask(data_mask_reg),
        .i_data_write(data_in_muxed_reg),
        .o_data_read(read_word),
        .o_finish(transmitter_finish),
        .o_need_next_word(spi_write_next_byte),
        .o_recieved_next_word(spi_read_next_byte),

        // SPI Pins
        .o_bus_clock(o_bus_clock),
        .o_bus_clock_neg(o_bus_clock_neg),
        .o_chip_select_neg(o_chip_select_neg),

        .i_data_strobe(i_data_strobe),
        .o_data_strobe(o_data_strobe),
        .o_data_strobe_output_enable(o_data_strobe_output_enable),

        .i_data(i_data),
        .o_data(o_data),
        .o_data_output_enable(o_data_output_enable)
    );


    assign start_transmission = (control_state_reg != IDLE && control_state_reg != WAIT);
    assign config_read_data = (control_state_reg == READ);
    assign config_write_data = (control_state_reg == WRITE || control_state_reg == WRITE_CONFIG);
    assign config_write_address = (control_state_reg == READ || control_state_reg == WRITE || control_state_reg == WRITE_CONFIG);
    assign o_busy = (control_state_reg != IDLE) | ~init_done;
    assign o_data_read = data_read_reg;


    integer delay_fsm;  // ToDo: make this more beautifull - the state machine probably needs multiple delays. (issue #10)
    always @(*) begin : control_logic
        address_nxt = address_reg;
        opcode_nxt = opcode_reg;
        control_state_nxt = control_state_reg;

        case (control_state_reg)
            IDLE: begin
                if (config_start) begin
                    address_nxt = ADDRESS_LENGTH'(CONFIG_ADDRESS);
                    opcode_nxt = HAS_WRITE_ENABLE ? OPCODE_WRITE_ENABLE : OPCODE_WRITE_ANY_REG;
                    control_state_nxt = HAS_WRITE_ENABLE ? WRITE_ENABLE : WRITE_CONFIG;
                end else if (go) begin
                    address_nxt = i_address;
                    if (!i_write_enable) begin
                        opcode_nxt = OPCODE_READ;
                        control_state_nxt = READ;
                    end else begin
                        opcode_nxt = HAS_WRITE_ENABLE ? OPCODE_WRITE_ENABLE : OPCODE_WRITE;
                        control_state_nxt = HAS_WRITE_ENABLE ? WRITE_ENABLE : WRITE;
                    end
                end
            end

            READ: begin
                if (transmitter_finish) control_state_nxt = IDLE;
            end

            WRITE_ENABLE: begin
                if (transmitter_finish) begin
                    control_state_nxt = WAIT;
                end
            end

            WAIT: begin
                if (delay_fsm == DELAY_CYCLES) begin
                    control_state_nxt = (config_is_config_operation) ? WRITE_CONFIG : WRITE;
                    opcode_nxt = (config_is_config_operation) ? OPCODE_WRITE_ANY_REG : OPCODE_WRITE;
                end
            end

            WRITE: begin
                if (transmitter_finish) control_state_nxt = IDLE;
            end

            WRITE_CONFIG: begin
                if (transmitter_finish) control_state_nxt = IDLE;
            end

            default: control_state_nxt = IDLE;
        endcase

    end


    always @(posedge clk or negedge reset_neg) begin : control_register
        if (!reset_neg) begin
            control_state_reg <= IDLE;
            address_reg <= 0;
            opcode_reg <= 0;
            config_is_config_operation <= 1'b0;
            init_done <= 1'b0;
            cem_counter <= 0;
            delay_fsm <= 0;
        end else begin
            address_reg <= address_nxt;
            opcode_reg <= opcode_nxt;
            control_state_reg <= control_state_nxt;
            delay_fsm <= 0;
            if (control_state_reg == IDLE) cem_counter <= 0;
            else if (!cem_timeout) cem_counter <= cem_counter + 1;

            if (control_state_reg == WAIT) begin
                delay_fsm <= delay_fsm + 1;
            end

            if (config_start) config_is_config_operation <= 1'b1;
            if (control_state_reg == WRITE_CONFIG && transmitter_finish) begin
                config_is_config_operation <= 1'b0;
                init_done <= 1'b1;
            end
        end
    end


    always @(*) begin : data_logic
        last_word_nxt = last_word_reg;
        data_in_muxed_nxt = data_in_muxed_reg;
        data_mask_nxt = data_mask_reg;
        read_next_word_nxt = 0;
        write_next_word_nxt = 0;
        config_data_nxt = config_data_reg;
        write_stop_nxt = write_stop_reg;

        // to get the first data byte
        if (control_state_reg == IDLE) begin
            if (config_start) begin
                data_in_muxed_nxt = config_data_reg;
                data_mask_nxt = 0;
                last_word_nxt = 1'b1;
            end else if (go) begin
                data_in_muxed_nxt = i_data_write;
                data_mask_nxt = i_data_word_mask;
                write_next_word_nxt = 1;    // save the data and request the next data on the bus
                last_word_nxt = i_last_word;
            end
            else
                last_word_nxt = 0;
            write_stop_nxt = 1'b0;
        end

        if (control_state_reg == WRITE | control_state_reg == READ | control_state_reg == WRITE_CONFIG) begin
            if (spi_write_next_byte | spi_read_next_byte) begin
                write_next_word_nxt = 1;     // ToDo: the DATA_BYTES can make problems, when it is too small (issue #10)
                read_next_word_nxt = 1;

                data_in_muxed_nxt = (config_is_config_operation) ? config_data_reg : i_data_write;
                data_mask_nxt = (config_is_config_operation) ? {STRB_WIDTH{1'b0}} : i_data_word_mask;
                // only set to 1, never reset to 0
                if (i_last_word)
                    last_word_nxt = 1'b1;

                // write without a beat (master stalls) or tCEM reached: send this word masked as the last one,
                // so the memory transaction ends on a full clock, the rest is written after a restart
                if (control_state_reg == WRITE && !last_word_reg && !write_word_valid) begin
                    data_mask_nxt = {STRB_WIDTH{1'b1}};
                    last_word_nxt = 1'b1;
                    write_stop_nxt = 1'b1;
                end

                // read: stop at tCEM, the AXI interface reads the rest after a restart
                if (control_state_reg == READ && cem_timeout) last_word_nxt = 1'b1;
            end
        end
    end

    // problem: when an unalligned read is done, some old data still resides in the read register
    // either it should be reset to 0 or masked out by the byte strobe thingy (issue #18)

    always @(posedge clk or negedge reset_neg) begin : data_register
        if (!reset_neg) begin
            data_in_muxed_reg <= 0;
            data_mask_reg <= 0;
            last_word_reg <= 0;
            data_read_reg <= 0;
            read_next_word_reg <= 0;
            write_next_word_reg <= 0;
            write_stop_reg <= 0;
            config_data_reg <= CONFIG_DATA;  // this is set only on reset - it should be more configurable (issue #16)
            // problem is, that on the go signal, the transmitter is started. So on the next clock edge it will sample the data.
            // if the config_data_nxt is set to the correct value, then it arrives a cycle late.
        end else begin
            data_in_muxed_reg <= data_in_muxed_nxt;
            data_mask_reg <= data_mask_nxt;
            last_word_reg <= last_word_nxt;
            read_next_word_reg <= read_next_word_nxt;
            write_next_word_reg <= write_next_word_nxt;
            config_data_reg <= config_data_nxt;
            write_stop_reg <= write_stop_nxt;

            if (spi_read_next_byte) begin
                data_read_reg <= read_word;
            end
        end
    end


endmodule
