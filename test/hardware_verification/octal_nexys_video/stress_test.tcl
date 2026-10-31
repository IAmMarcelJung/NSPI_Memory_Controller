# Memory stress test over JTAG-AXI, errors counted per chip (byte lane 0..3 = RAM A..D).
#   vivado -mode batch -source stress_test.tcl -tclargs <bitfile|-> <passes> <result file>
# Patterns per pass, each over STRESS_BYTES (written in 1 KB bursts, read back in 1 KB bursts):
#   toggle00ff: 00000000 / ffffffff per beat -> all DQ of every chip toggle on every clock edge
#   toggle55aa: 55555555 / aaaaaaaa per beat -> all DQ toggle, neighbours in opposite direction
#   walking:    one hot byte per beat (same on all chips)
#   random:     random words (new seed every pass)

set script_dir [file dirname [file normalize [info script]]]
set bitfile [lindex $argv 0]
set passes [lindex $argv 1]
set result_file [lindex $argv 2]
set STRESS_BYTES 65536
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

proc pattern_word {name i} {
    switch $name {
        toggle00ff { return [expr {$i % 2 ? 0xffffffff : 0x00000000}] }
        toggle55aa { return [expr {$i % 2 ? 0xaaaaaaaa : 0x55555555}] }
        walking    { set b [expr {1 << ($i % 8)}]; return [expr {$b | ($b << 8) | ($b << 16) | ($b << 24)}] }
        random     { return [expr {int(rand() * 4294967296.0)}] }
    }
}

set errors {0 0 0 0}
set words_checked 0
# on a mismatch the burst is read again 3 times: same wrong value = written wrong, changing = read path
set stable_errors 0
set changing_errors 0
set examples {}
array set pattern_errors {toggle00ff 0 toggle55aa 0 walking 0 random 0}

proc read_burst {axi addr len} {
    create_hw_axi_txn -force rr $axi -type read -address [format %08x $addr] -len $len
    run_hw_axi -quiet [get_hw_axi_txns rr]
    return [get_property DATA [get_hw_axi_txns rr]]
}
proc word_of {d i} {
    set n [string length $d]
    return [scan [string range $d [expr {$n - 8 * ($i + 1)}] [expr {$n - 8 * $i - 1}]] %x]
}
set t_start [clock seconds]
for {set pass 0} {$pass < $passes} {incr pass} {
    foreach name {toggle00ff toggle55aa walking random} {
        expr {srand(1000 + $pass)}
        array unset ref
        # write
        for {set addr 0} {$addr < $STRESS_BYTES} {incr addr [expr {4 * $BURST}]} {
            set data ""
            for {set i 0} {$i < $BURST} {incr i} {
                set w [pattern_word $name [expr {$addr / 4 + $i}]]
                set ref([expr {$addr / 4 + $i}]) $w
                set data "[format %08x $w]$data"
            }
            create_hw_axi_txn -force wr $axi -type write -address [format %08x $addr] -len $BURST -data $data
            run_hw_axi -quiet [get_hw_axi_txns wr]
        }
        # read back
        for {set addr 0} {$addr < $STRESS_BYTES} {incr addr [expr {4 * $BURST}]} {
            create_hw_axi_txn -force rd $axi -type read -address [format %08x $addr] -len $BURST
            if {[catch {run_hw_axi -quiet [get_hw_axi_txns rd]}]} {
                puts "read timeout at $addr"
                lset errors 0 [expr {[lindex $errors 0] + 1000000}]
                continue
            }
            set d [get_property DATA [get_hw_axi_txns rd]]
            set rereads {}
            for {set i 0} {$i < $BURST} {incr i} {
                set hw [word_of $d $i]
                set expected $ref([expr {$addr / 4 + $i}])
                set diff [expr {$hw ^ $expected}]
                incr words_checked
                if {$diff == 0} continue
                incr pattern_errors($name)
                for {set chip 0} {$chip < 4} {incr chip} {
                    if {($diff >> (8 * $chip)) & 0xff} { lset errors $chip [expr {[lindex $errors $chip] + 1}] }
                }
                if {$rereads eq ""} {
                    for {set r 0} {$r < 3} {incr r} { lappend rereads [read_burst $axi $addr $BURST] }
                }
                set again {}
                foreach rd_again $rereads { lappend again [format %08x [word_of $rd_again $i]] }
                if {[llength [lsort -unique [concat [format %08x $hw] $again]]] == 1} { incr stable_errors } else { incr changing_errors }
                if {[llength $examples] < 20} {
                    lappend examples "$name addr=[format %08x [expr {$addr + 4 * $i}]] exp=[format %08x $expected] got=[format %08x $hw] diff=[format %08x $diff] rereads=[join $again ,]"
                }
            }
        }
    }
    puts "pass $pass done, errors per chip A..D: $errors"
}
set seconds [expr {[clock seconds] - $t_start}]
set fh [open $result_file a]
puts $fh "[file tail $bitfile] passes=$passes words=$words_checked seconds=$seconds byte_errors_A_B_C_D=[join $errors ,] stable=$stable_errors changing=$changing_errors per_pattern=[array get pattern_errors]"
foreach e $examples { puts $fh "    $e" }
close $fh
puts "RESULT [file tail $bitfile] words=$words_checked errors=$errors"
