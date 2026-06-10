package org.firstinspires.ftc.teamcode.core.runtime

/**
 * Shared Ivy priority ladder. Higher interrupts lower; gaps left so a season
 * fork can slot levels in between without renumbering.
 *
 *  - [DEFAULT] — subsystem default commands; quietly lose to explicit work.
 *  - [AUTON_ROUTINE] — autonomous routines and teleop auto-assists
 *    (auto-aim, auto-align). Interrupt defaults only.
 *  - [DRIVER_ACTION] — direct driver-triggered actions. Deliberately above
 *    [AUTON_ROUTINE] so a driver input always wins over an assist.
 *  - [DRIVER_OVERRIDE] — panic / manual-override bindings that must preempt
 *    everything (e.g. "stop the lift NOW").
 */
object CommandPriorities {
    const val DEFAULT: Int = 0
    const val AUTON_ROUTINE: Int = 10
    const val DRIVER_ACTION: Int = 20
    const val DRIVER_OVERRIDE: Int = 30
}
