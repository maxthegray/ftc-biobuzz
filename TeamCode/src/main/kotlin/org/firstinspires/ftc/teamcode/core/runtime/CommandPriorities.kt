package org.firstinspires.ftc.teamcode.core.runtime

/**
 * Shared Ivy priority convention.
 *
 * Defaults stay at 0 so they quietly lose to explicit work. Driver-triggered
 * actions and autonomous routines use 10 so they interrupt defaults and
 * arbitrate evenly with each other by Ivy's normal conflict behavior.
 */
object CommandPriorities {
    const val DEFAULT: Int = 0
    const val DRIVER_ACTION: Int = 10
    const val AUTON_ROUTINE: Int = 10
}
