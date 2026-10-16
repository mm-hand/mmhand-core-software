# GDB's detach message does not guarantee that both ESP SMP cores resumed.
# Install after the target configuration. No reset and no motor authorization.
proc mmhand_debug_release {} {
    set mm_targets [target names]
    foreach t $mm_targets {
        targets $t
        if {[$t curstate] eq "running"} { halt }
    }
    # All cores must be free of break/watchpoints BEFORE any core is resumed.
    foreach t $mm_targets {
        targets $t
        set mm_error ""
        if {[catch {rbp all} mm_message]} { append mm_error $mm_message }
        if {[catch {rwp all} mm_message]} { append mm_error $mm_message }
        if {[string match "esp32p4.hp.cpu*" $t]} {
            # P4 has three hardware triggers. A killed GDB can leave OpenOCD's
            # breakpoint bookkeeping unable to remove them. Clear and read
            # back the actual hardware, including orphaned triggers from an
            # earlier server. Register TYPE bits remain read-only.
            for {set mm_slot 0} {$mm_slot < 3} {incr mm_slot} {
                reg tselect $mm_slot
                reg tdata1 0
                reg tdata2 0
                set mm_control [dict get [get_reg -force {tdata1}] tdata1]
                if {($mm_control & 0x7) != 0} {
                    error "$t trigger $mm_slot remains enabled"
                }
            }
            reg tselect 0
            if {$mm_error ne ""} {
                echo "MMHAND: hardware triggers cleared; restart this OpenOCD server before further debugging ($mm_error)"
            }
        } elseif {$mm_error ne ""} {
            error $mm_error
        }
    }
    foreach t $mm_targets {
        targets $t
        if {[$t curstate] eq "halted"} { resume }
    }
}

foreach mm_target [target names] {
    $mm_target configure -event gdb-detach { mmhand_debug_release }
}
