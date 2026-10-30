# Write bitstreams with different DRIVE / SLEW on the PSRAM clock output from the routed checkpoint.
#   vivado -mode batch -source slew_variants.tcl      (needs build/top_quad_routed.dcp from build_quad.tcl)
# All four PSRAMs share one CLK net on the FMC board, so every variant affects all chips.
set script_dir [file dirname [file normalize [info script]]]
set negedge [expr {[info exists ::env(BUS_CLOCK_NEGEDGE)] && $::env(BUS_CLOCK_NEGEDGE) eq "1"}]
set build_dir [expr {$negedge ? "$script_dir/build_negedge" : "$script_dir/build"}]
open_checkpoint $build_dir/top_quad_routed.dcp
set clk_port [get_ports o_spi_bus_clock]
puts "default: DRIVE [get_property DRIVE $clk_port] SLEW [get_property SLEW $clk_port]"

set fh [open $build_dir/slew_timing.txt w]
foreach slew {SLOW FAST} {
    foreach drive {4 8 12 16} {
        set_property DRIVE $drive $clk_port
        set_property SLEW $slew $clk_port
        # IO timing of the PSRAM interface with this clock buffer (clock path delay changes with DRIVE/SLEW)
        set setup [get_property SLACK [get_timing_paths -delay_type max -to [get_clocks bus_clk] -max_paths 1]]
        set hold [get_property SLACK [get_timing_paths -delay_type min -to [get_clocks bus_clk] -max_paths 1]]
        set setup_in [get_property SLACK [get_timing_paths -delay_type max -from [get_clocks bus_clk] -max_paths 1]]
        set hold_in [get_property SLACK [get_timing_paths -delay_type min -from [get_clocks bus_clk] -max_paths 1]]
        puts $fh "drive=$drive slew=$slew out_setup=$setup out_hold=$hold in_setup=$setup_in in_hold=$hold_in"
        write_bitstream -force $build_dir/slew_${drive}_${slew}.bit
    }
}
close $fh
