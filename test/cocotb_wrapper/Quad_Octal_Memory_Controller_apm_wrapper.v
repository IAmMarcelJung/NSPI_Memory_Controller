`timescale 1ps / 1ps

module Quad_Octal_Memory_Controller_apm_wrapper #(
    // Width of data bus in bits
    parameter DATA_WIDTH = 32,
    // Width of address bus in bits
    parameter ADDR_WIDTH = 32,
    // Width of wstrb (width of data bus in words)
    parameter STRB_WIDTH = (DATA_WIDTH / 8),
    // Width of ID signal
    parameter ID_WIDTH = 8,
    // Extra pipeline register on output
    parameter PIPELINE_OUTPUT = 1'b0,
    // memory model
    parameter integer MEM_ADDR_BITS = 16,
    parameter integer REFRESH_PUSHOUT_PERCENT = 0,
    parameter integer T_PU = 150_000_000,
    parameter integer INIT_BYTE = -1,
    parameter integer CEM_MAX_CYCLES = 350,
    parameter BUS_CLOCK_NEGEDGE = 1,
    // test only: R channel FIFO between controller and master (0 = direct), to estimate what a skid buffer would gain
    parameter integer R_FIFO_DEPTH = 0
) (
    input clk,
    input reset,

    // AXI Pins
    input  wire [  ID_WIDTH-1:0] s_axi_awid,     // write address channel
    input  wire [ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire [           7:0] s_axi_awlen,
    input  wire [           2:0] s_axi_awsize,
    input  wire [           1:0] s_axi_awburst,
    input  wire                  s_axi_awlock,
    input  wire [           3:0] s_axi_awcache,
    input  wire [           2:0] s_axi_awprot,
    input  wire                  s_axi_awvalid,
    output wire                  s_axi_awready,

    input  wire [DATA_WIDTH-1:0] s_axi_wdata,   // write data channel
    input  wire [STRB_WIDTH-1:0] s_axi_wstrb,
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
);

    integer test_number = 0;
    wire bus_clock, bus_clock_neg;
    wire spi_reset_neg;
    wire [STRB_WIDTH-1:0] chip_select_neg;

    wire [STRB_WIDTH-1:0] control_data_output_enable;
    wire [STRB_WIDTH-1:0] control_strobe_output_enable;

    wire [DATA_WIDTH-1:0] mem_data;
    wire [STRB_WIDTH-1:0] mem_data_strobe;

    wire [DATA_WIDTH-1:0] control_i_data;
    wire [DATA_WIDTH-1:0] control_o_data;
    wire [STRB_WIDTH-1:0] control_i_data_strobe;
    wire [STRB_WIDTH-1:0] control_o_data_strobe;

    localparam BYTE = 8;
    genvar channel;
    generate
        for (channel = 0; channel < 4; channel = channel + 1) begin : gen_signals
            assign #1 mem_data[channel*BYTE +: BYTE] = (control_data_output_enable[channel]) ? control_o_data[channel*BYTE +: BYTE] : 8'bz;
            assign control_i_data[channel*BYTE+:BYTE] = mem_data[channel*BYTE+:BYTE];

            assign #1 mem_data_strobe[channel] = (control_strobe_output_enable[channel]) ? control_o_data_strobe[channel] : 1'bz;
            assign control_i_data_strobe[channel] = mem_data_strobe[channel];
        end
    endgenerate



    localparam INFINEON_CHIP = 1;
    localparam APMEMORY_CHIP = 2;

    // R channel between the controller and the (optional) FIFO
    wire [  ID_WIDTH-1:0] ctrl_rid;
    wire [DATA_WIDTH-1:0] ctrl_rdata;
    wire [           1:0] ctrl_rresp;
    wire                  ctrl_rlast;
    wire                  ctrl_rvalid;
    wire                  ctrl_rready;

    generate
        if (R_FIFO_DEPTH == 0) begin : gen_r_direct
            assign s_axi_rid = ctrl_rid;
            assign s_axi_rdata = ctrl_rdata;
            assign s_axi_rresp = ctrl_rresp;
            assign s_axi_rlast = ctrl_rlast;
            assign s_axi_rvalid = ctrl_rvalid;
            assign ctrl_rready = s_axi_rready;
        end else begin : gen_r_fifo
            localparam integer W = ID_WIDTH + DATA_WIDTH + 3;
            localparam integer PW = $clog2(R_FIFO_DEPTH) + 1;
            reg [W-1:0] fifo_mem[0:R_FIFO_DEPTH-1];
            reg [PW-1:0] count;
            reg [PW-2:0] wr_ptr, rd_ptr;
            wire push = ctrl_rvalid & ctrl_rready;
            wire pop = s_axi_rvalid & s_axi_rready;
            assign ctrl_rready = (count < R_FIFO_DEPTH);
            assign s_axi_rvalid = (count != 0);
            assign {s_axi_rid, s_axi_rdata, s_axi_rresp, s_axi_rlast} = fifo_mem[rd_ptr];
            always @(posedge clk) begin
                if (push) begin
                    fifo_mem[wr_ptr] <= {ctrl_rid, ctrl_rdata, ctrl_rresp, ctrl_rlast};
                    wr_ptr <= (wr_ptr == R_FIFO_DEPTH - 1) ? 0 : wr_ptr + 1;
                end
                if (pop) rd_ptr <= (rd_ptr == R_FIFO_DEPTH - 1) ? 0 : rd_ptr + 1;
                count <= count + push - pop;
                if (reset) begin
                    count <= 0;
                    wr_ptr <= 0;
                    rd_ptr <= 0;
                end
            end
        end
    endgenerate

    OctalMemoryController_quad #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .CHIP_TYPE (APMEMORY_CHIP),
        .MEMORY_RESET_CYCLES(20000),  // 400 us, the model needs no supply ramp
        .CEM_MAX_CYCLES(CEM_MAX_CYCLES),
        .BUS_CLOCK_NEGEDGE(BUS_CLOCK_NEGEDGE)
    ) Controller (
        .clk  (clk),
        .rstn(~reset),

        // SPI Pins
        .o_spi_bus_clock(bus_clock),
        .o_spi_bus_clock_neg(bus_clock_neg),
        .o_spi_chip_select_neg(chip_select_neg),
        .o_spi_reset_neg(spi_reset_neg),

        .i_data_strobe(control_i_data_strobe),
        .o_data_strobe(control_o_data_strobe),
        .o_data_strobe_output_enable(control_strobe_output_enable),

        .i_data(control_i_data),
        .o_data(control_o_data),
        .o_data_output_enable(control_data_output_enable),


        // AXI Pins
        .s_axi_awid(s_axi_awid),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awlen(s_axi_awlen),
        .s_axi_awsize(s_axi_awsize),
        .s_axi_awburst(s_axi_awburst),
        .s_axi_awlock(s_axi_awlock),
        .s_axi_awcache(s_axi_awcache),
        .s_axi_awprot(s_axi_awprot),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wstrb(s_axi_wstrb),
        .s_axi_wlast(s_axi_wlast),
        .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready),
        .s_axi_bid(s_axi_bid),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_arid(s_axi_arid),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_arlen(s_axi_arlen),
        .s_axi_arsize(s_axi_arsize),
        .s_axi_arburst(s_axi_arburst),
        .s_axi_arlock(s_axi_arlock),
        .s_axi_arcache(s_axi_arcache),
        .s_axi_arprot(s_axi_arprot),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rid(ctrl_rid),
        .s_axi_rdata(ctrl_rdata),
        .s_axi_rresp(ctrl_rresp),
        .s_axi_rlast(ctrl_rlast),
        .s_axi_rvalid(ctrl_rvalid),
        .s_axi_rready(ctrl_rready)
    );


    genvar index;
    generate
        for (index = 0; index < 4; index = index + 1) begin : gen_memory
            aps12808l_model #(
                .ADDR_BITS(MEM_ADDR_BITS),
                .REFRESH_PUSHOUT_PERCENT(REFRESH_PUSHOUT_PERCENT),
                .SEED(index + 1),
                .T_PU(T_PU),
                .INIT_BYTE(INIT_BYTE)
            ) Memory (
                .CE_n(chip_select_neg[index]),
                .CLK(bus_clock),
                .RESET_n(spi_reset_neg),
                .DQ(mem_data[index*BYTE+:BYTE]),
                .DQS(mem_data_strobe[index])
            );
        end
    endgenerate

endmodule
