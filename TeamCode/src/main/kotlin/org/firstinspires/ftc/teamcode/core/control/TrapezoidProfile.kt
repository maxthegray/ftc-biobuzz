package org.firstinspires.ftc.teamcode.core.control

import kotlin.math.sqrt

/** A position + velocity sample along a motion profile. */
data class ProfileState(
    @JvmField val position: Double,
    @JvmField val velocity: Double = 0.0,
)

/**
 * Velocity/acceleration limits for a [TrapezoidProfile]. Mutable so a season
 * fork can hold these in a Panels `@Configurable` object and tune them live.
 */
class ProfileConstraints(
    maxVelocity: Double,
    maxAcceleration: Double,
) {
    @JvmField var maxVelocity: Double = maxVelocity
    @JvmField var maxAcceleration: Double = maxAcceleration
}

/**
 * Trapezoidal motion profile, evaluated incrementally: each call advances the
 * previous setpoint by [calculate]'s `dtSeconds` toward the goal under the
 * [constraints]. Because the profile is re-solved from the current setpoint
 * every tick, changing the goal mid-motion just re-plans from wherever the
 * setpoint is — no explicit re-profiling step.
 *
 * Mid-motion reversals are handled physically (unlike the classic
 * WPILib-style formulation, which teleports the velocity): a setpoint moving
 * away from the goal brakes to rest at maxAcceleration first, and a setpoint
 * moving toward the goal too fast to stop decelerates at the limit,
 * overshoots, and comes back. Acceleration and continuity bounds hold under
 * arbitrary goal changes — `TrapezoidProfilePropertyTest` enforces this.
 *
 * Constraints are read on every call, so live tuning takes effect immediately.
 */
class TrapezoidProfile(private val constraints: ProfileConstraints) {

    /** Advance [current] by [dtSeconds] toward [goal]. Returns the new setpoint. */
    fun calculate(dtSeconds: Double, current: ProfileState, goal: ProfileState): ProfileState {
        val maxVelocity = constraints.maxVelocity
        val maxAcceleration = constraints.maxAcceleration
        require(maxVelocity > 0.0 && maxAcceleration > 0.0) {
            "Profile constraints must be positive (maxVelocity=$maxVelocity, maxAcceleration=$maxAcceleration)"
        }
        if (dtSeconds <= 0.0) return current

        // Solve in a frame where the goal is ahead of the setpoint.
        val direction = if (current.position > goal.position) -1.0 else 1.0
        var setpoint = flip(current, direction)
        val target = flip(goal, direction)
        var dt = dtSeconds

        // Velocity opposing the direction of travel (a goal re-targeted
        // behind a moving setpoint): brake to rest at maxAcceleration first.
        // The trapezoid math below models the setpoint as a point on a
        // forward-accelerating profile and would otherwise teleport the
        // velocity discontinuously.
        if (setpoint.velocity < 0.0) {
            val brakingTime = -setpoint.velocity / maxAcceleration
            if (dt <= brakingTime) {
                val v = setpoint.velocity + maxAcceleration * dt
                val x = setpoint.position + (setpoint.velocity + v) * 0.5 * dt
                return flip(ProfileState(x, v), direction)
            }
            val restPosition =
                setpoint.position + setpoint.velocity * brakingTime / 2.0
            setpoint = ProfileState(restPosition, 0.0)
            dt -= brakingTime
            if (dt <= 0.0) return flip(setpoint, direction)
        }

        if (setpoint.velocity > maxVelocity) {
            setpoint = ProfileState(setpoint.position, maxVelocity)
        }

        // Extend the profile backward/forward to zero velocity at both ends,
        // then cut off the parts before the setpoint and after the goal.
        val cutoffBegin = setpoint.velocity / maxAcceleration
        val cutoffDistBegin = cutoffBegin * cutoffBegin * maxAcceleration / 2.0
        val cutoffEnd = target.velocity / maxAcceleration
        val cutoffDistEnd = cutoffEnd * cutoffEnd * maxAcceleration / 2.0

        val fullTrapezoidDist =
            cutoffDistBegin + (target.position - setpoint.position) + cutoffDistEnd
        var accelerationTime = maxVelocity / maxAcceleration
        var fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * maxAcceleration
        if (fullSpeedDist < 0.0) {
            // Triangular profile: never reaches max velocity.
            accelerationTime = sqrt((fullTrapezoidDist / maxAcceleration).coerceAtLeast(0.0))
            fullSpeedDist = 0.0
        }

        val endAccel = accelerationTime - cutoffBegin

        // endAccel < 0 means the current velocity is above the synthetic
        // profile's peak: the setpoint is moving toward the goal too fast to
        // stop in time. The trapezoid cannot represent the unavoidable
        // overshoot (it is monotonic), so handle it physically — decelerate
        // at the limit, overshoot, and let the opposing-velocity branch
        // above bring it back on later ticks.
        if (endAccel < 0.0) {
            val brakingTime = setpoint.velocity / maxAcceleration
            val result = if (dt < brakingTime) {
                val v = setpoint.velocity - maxAcceleration * dt
                ProfileState(setpoint.position + (setpoint.velocity + v) * 0.5 * dt, v)
            } else {
                ProfileState(setpoint.position + setpoint.velocity * brakingTime * 0.5, 0.0)
            }
            return flip(result, direction)
        }

        val endFullSpeed = endAccel + fullSpeedDist / maxVelocity
        val endDecel = endFullSpeed + accelerationTime - cutoffEnd

        val t = dt
        val result = when {
            t < endAccel -> ProfileState(
                setpoint.position + (setpoint.velocity + t * maxAcceleration / 2.0) * t,
                setpoint.velocity + t * maxAcceleration,
            )
            t < endFullSpeed -> ProfileState(
                setpoint.position +
                    (setpoint.velocity + endAccel * maxAcceleration / 2.0) * endAccel +
                    maxVelocity * (t - endAccel),
                maxVelocity,
            )
            t <= endDecel -> {
                val timeLeft = endDecel - t
                ProfileState(
                    target.position - (target.velocity + timeLeft * maxAcceleration / 2.0) * timeLeft,
                    target.velocity + timeLeft * maxAcceleration,
                )
            }
            else -> target
        }
        return flip(result, direction)
    }

    private fun flip(state: ProfileState, direction: Double): ProfileState =
        if (direction == 1.0) state else ProfileState(state.position * direction, state.velocity * direction)
}
