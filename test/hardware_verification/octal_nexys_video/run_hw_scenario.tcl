# Program the board (optional) and run the scenario from hw_scenario.py over JTAG-AXI.
#   python3 hw_scenario.py tcl > build/hw_scenario.tcl
#   vivado -mode batch -source run_hw_scenario.tcl -tclargs [program]
set script_dir [file dirname [file normalize [info script]]]
set build_dir $script_dir/build
set result_file $build_dir/hw_result.txt

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
if {[lindex $argv 0] eq "program"} {
    set_property PROGRAM.FILE $build_dir/top_quad.bit $device
    set_property PROBES.FILE $build_dir/top_quad.ltx $device
    program_hw_devices $device
    # the controller holds RESET# of the memories low for 400 us after reset
    after 100
}
refresh_hw_device $device
puts "JTAG-AXI masters: [get_hw_axis]"
source $build_dir/hw_scenario.tcl
puts "result written to $result_file"
