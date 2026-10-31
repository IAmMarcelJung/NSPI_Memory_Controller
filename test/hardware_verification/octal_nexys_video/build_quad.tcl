# Build the quad APMemory controller for the Nexys Video (xc7a200t) with the FMC PSRAM board.
#
#   vivado -mode batch -source build_quad.tcl                 -> build/top_quad.bit
#   vivado -mode batch -source build_quad.tcl -tclargs program -> build and program the board
#
# The JTAG-AXI master is then used with axi_verification_test.tcl / hw_test_quad.tcl.

set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file normalize $script_dir/../../..]
# BUS_CLOCK_NEGEDGE=1 in the environment: bus clock from the negedge register (test variant), own build directory
set negedge [expr {[info exists ::env(BUS_CLOCK_NEGEDGE)] && $::env(BUS_CLOCK_NEGEDGE) eq "1"}]
set build_dir [expr {$negedge ? "$script_dir/build_negedge" : "$script_dir/build"}]
set part xc7a200tsbg484-1

file mkdir $build_dir/ip
create_project -in_memory -part $part
set_property target_language Verilog [current_project]

# IPs
create_ip -name clk_wiz -vendor xilinx.com -library ip -module_name clk_wiz_0 -dir $build_dir/ip
set_property -dict [list \
    CONFIG.PRIM_IN_FREQ {100.000} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {50.000} \
    CONFIG.USE_LOCKED {true} \
    CONFIG.USE_RESET {true} \
    CONFIG.RESET_TYPE {ACTIVE_HIGH} \
] [get_ips clk_wiz_0]

create_ip -name jtag_axi -vendor xilinx.com -library ip -module_name jtag_axi_0 -dir $build_dir/ip
set_property -dict [list \
    CONFIG.PROTOCOL {0} \
    CONFIG.M_AXI_DATA_WIDTH {32} \
    CONFIG.M_AXI_ADDR_WIDTH {32} \
    CONFIG.M_AXI_ID_WIDTH {1} \
] [get_ips jtag_axi_0]

generate_target all [get_ips]
synth_ip [get_ips]

# sources
# AXIInterface.v last: it sets `default_nettype none without resetting it afterwards
read_verilog [list \
    $repo_dir/src/OctalSPI_quad.v \
    $repo_dir/src/OSPIController_quad.v \
    $repo_dir/src/OctalMemoryController_quad.v \
    $script_dir/top_quad.v \
    $repo_dir/src/AXIInterface.v \
]
set_property file_type SystemVerilog [get_files *.v]

read_xdc $script_dir/Nexys-Video-Master_Quad.xdc
foreach chip {A B C D} {
    if {$negedge} {
        # the bus clock comes from bus_clock_neg_reg: it toggles on the falling edges of the system clock
        # (its C pin is inverted, so edges 1 3 5 at that pin are the falling edges of sys_clk)
        set fh [open $script_dir/Timing_RAM_$chip.xdc]
        set xdc [read $fh]
        close $fh
        regsub -line {^create_generated_clock .*$} $xdc {create_generated_clock -name bus_clk -source [get_pins -of_objects [get_cells -hier -filter {NAME =~ *SPI_Transmitter/bus_clock_neg_reg_reg}] -filter {REF_PIN_NAME == C}] -edges {1 3 5} [get_ports o_spi_bus_clock]} xdc
        set fh [open $build_dir/Timing_RAM_$chip.xdc w]
        puts $fh $xdc
        close $fh
        read_xdc $build_dir/Timing_RAM_$chip.xdc
    } else {
        read_xdc $script_dir/Timing_RAM_$chip.xdc
    }
}

synth_design -top top_quad -part $part -generic BUS_CLOCK_NEGEDGE=$negedge
report_utilization -file $build_dir/utilization_synth.rpt
opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force $build_dir/top_quad_routed.dcp
report_timing_summary -max_paths 20 -file $build_dir/timing_summary.rpt
report_drc -file $build_dir/drc.rpt
write_bitstream -force $build_dir/top_quad.bit
write_debug_probes -force $build_dir/top_quad.ltx

if {[lindex $argv 0] eq "program"} {
    open_hw_manager
    connect_hw_server
# the Digilent adapter can show up as more than one target, take the one with the FPGA
proc open_fpga_target {} {
    upvar device device
    foreach target [get_hw_targets] {
        if {[catch {open_hw_target $target}]} {
            catch {close_hw_target $target}  ;# a failed open leaves the target open
            continue
        }
        set device [lindex [get_hw_devices -quiet xc7a200t*] 0]
        if {$device ne ""} return
        close_hw_target $target
    }
    error "no xc7a200t found on any JTAG target"
}
    open_fpga_target
    current_hw_device $device
    set_property PROGRAM.FILE $build_dir/top_quad.bit $device
    program_hw_devices $device
    refresh_hw_device $device
    puts "Programmed. JTAG-AXI masters: [get_hw_axis]"
}
