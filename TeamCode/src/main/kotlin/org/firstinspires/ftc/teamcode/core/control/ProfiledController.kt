package org.firstinspires.ftc.teamcode.core.control

import kotlin.math.abs

/**
 * Trapezoid profile + PIDF in one unit: the profile advances an internal
 * setpoint toward the goal each tick, and the PIDF tracks that setpoint with
 * velocity feedforward. This is the season-agnostic core of every profiled
 * mechanism (lift, arm, turret); [org.firstinspires.ftc.teamcode.core.subsystems.ProfiledMotorSubsystem]
 * wraps it around a single motor.
 *
 * Pure logic — no hardware, no clock — so it is fully host-testable.
 */
class ProfiledController(
    constraints: ProfileConstraints,
    gains: PIDFGains,
) {
    private val profile = TrapezoidProfile(constraints)
    private val pidf = PIDFController(gains)

    var setpoint = ProfileState(0.0)
        private set
    var goal = ProfileState(0.0)
        private set

    /**
     * Seed the profile at the mechanism's measured state. Call before the
     * first [update] after enabling closed-loop control, so the profile
     * starts from reality instead of stale state.
     */
    fun reset(measuredPosition: Double, measuredVelocity: Double = 0.0) {
        setpoint = ProfileState(measuredPosition, measuredVelocity)
        goal = setpoint
        pidf.reset()
    }

    fun setGoal(position: Double, velocity: Double = 0.0) {
        goal = ProfileState(position, velocity)
    }

    /** One control step: advance the profile, then track it. Returns the output. */
    fun update(dtSeconds: Double, measuredPosition: Double): Double {
        setpoint = profile.calculate(dtSeconds, setpoint, goal)
        return pidf.calculate(dtSeconds, measuredPosition, setpoint.position, setpoint.velocity)
    }

    /** True when the profile has finished and the mechanism is within [toleranceUnits] of the goal. */
    fun atGoal(measuredPosition: Double, toleranceUnits: Double): Boolean =
        abs(setpoint.position - goal.position) < 1e-9 &&
            abs(setpoint.velocity - goal.velocity) < 1e-9 &&
            abs(measuredPosition - goal.position) <= toleranceUnits
}
