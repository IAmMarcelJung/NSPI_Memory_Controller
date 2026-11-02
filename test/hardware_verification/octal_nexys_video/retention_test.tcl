# Retention / refresh test over JTAG-AXI: fill the memory, wait idle, read everything back.
#   vivado -mode batch -source retention_test.tcl -tclargs <bitfile|-> <wait minutes> <result file> [MB] [patterns]
#   e.g. ... -tclargs build/top_quad.bit 20 build/retention.txt 64 "hash inv"
# While the controller is idle CE# stays high and the memories refresh themselves; nothing else touches them.
# The data is a function of the address (no reference copy needed for 64 MB):
#   hash: address hash, every bit about half 0 / half 1
#   inv:  the same inverted, so every cell has held both values after both patterns
# JTAG moves ~130 KB/s: 64 MB take ~8 min per direction, plus the wait.
# Errors are counted per chip (byte lane 0..3 = RAM A..D).

set bitfile [lindex $argv 0]
set wait_minutes [lindex $argv 1]
set result_file [lindex $argv 2]
set megabytes [expr {[llength $argv] > 3 ? [lindex $argv 3] : 64}]
set patterns [expr {[llength $argv] > 4 ? [lindex $argv 4] : "hash inv"}]
set TOTAL_BYTES [expr {$megabytes * 1024 * 1024}]
set BURST 256

open_hw_manager
connect_hw_server
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
if {$bitfile ne "-"} {
    set_property PROGRAM.FILE $bitfile $device
    set_property PROBES.FILE {} $device
    program_hw_devices $device
    # the controller holds RESET# of the memories low for 20 ms after reset
    after 100
}
refresh_hw_device $device
set axi [lindex [get_hw_axis] 0]
reset_hw_axi $axi

proc pattern_word {name word_index} {
    set w [expr {(($word_index * 2654435761) ^ ($word_index >> 7) ^ 0x5a5a1234) & 0xffffffff}]
    if {$name eq "inv"} { set w [expr {$w ^ 0xffffffff}] }
    return $w
}
proc word_of {d i} {
    set n [string length $d]
    return [scan [string range $d [expr {$n - 8 * ($i + 1)}] [expr {$n - 8 * $i - 1}]] %x]
}

set fh [open $result_file a]
puts $fh "retention [file tail $bitfile] MB=$megabytes wait=${wait_minutes}min patterns=$patterns start=[clock format [clock seconds]]"
close $fh

set total_errors {0 0 0 0}
foreach name $patterns {
    set t0 [clock seconds]
    for {set addr 0} {$addr < $TOTAL_BYTES} {incr addr [expr {4 * $BURST}]} {
        set data ""
        for {set i 0} {$i < $BURST} {incr i} {
            set data "[format %08x [pattern_word $name [expr {$addr / 4 + $i}]]]$data"
        }
        create_hw_axi_txn -force wr $axi -type write -address [format %08x $addr] -len $BURST -data $data
        run_hw_axi -quiet [get_hw_axi_txns wr]
        if {$addr % (4 * 1024 * 1024) == 0} { puts "$name: written [expr {$addr >> 20}] MB" }
    }
    set t_write [expr {[clock seconds] - $t0}]
    puts "$name: write done in $t_write s, waiting $wait_minutes min"
    after [expr {int($wait_minutes * 60000)}]

    set t0 [clock seconds]
    set errors {0 0 0 0}
    set words 0
    set examples {}
    for {set addr 0} {$addr < $TOTAL_BYTES} {incr addr [expr {4 * $BURST}]} {
        create_hw_axi_txn -force rd $axi -type read -address [format %08x $addr] -len $BURST
        if {[catch {run_hw_axi -quiet [get_hw_axi_txns rd]}]} {
            puts "read timeout at [format %08x $addr]"
            lset errors 0 [expr {[lindex $errors 0] + 1000000}]
            continue
        }
        set d [get_property DATA [get_hw_axi_txns rd]]
        for {set i 0} {$i < $BURST} {incr i} {
            set expected [pattern_word $name [expr {$addr / 4 + $i}]]
            set diff [expr {[word_of $d $i] ^ $expected}]
            incr words
            if {$diff == 0} continue
            for {set chip 0} {$chip < 4} {incr chip} {
                if {($diff >> (8 * $chip)) & 0xff} { lset errors $chip [expr {[lindex $errors $chip] + 1}] }
            }
            if {[llength $examples] < 20} {
                lappend examples "$name addr=[format %08x [expr {$addr + 4 * $i}]] exp=[format %08x $expected] diff=[format %08x $diff]"
            }
        }
        if {$addr % (4 * 1024 * 1024) == 0} { puts "$name: read [expr {$addr >> 20}] MB, errors A..D: $errors" }
    }
    set t_read [expr {[clock seconds] - $t0}]
    for {set chip 0} {$chip < 4} {incr chip} {
        lset total_errors $chip [expr {[lindex $total_errors $chip] + [lindex $errors $chip]}]
    }
    set fh [open $result_file a]
    puts $fh "- $name words=$words write_s=$t_write wait_min=$wait_minutes read_s=$t_read byte_errors_A_B_C_D=[join $errors ,]"
    foreach e $examples { puts $fh "    $e" }
    close $fh
}
puts "RESULT retention errors A..D: $total_errors"
