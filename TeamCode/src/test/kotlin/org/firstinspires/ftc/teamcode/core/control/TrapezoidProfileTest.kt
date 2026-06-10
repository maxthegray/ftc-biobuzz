package org.firstinspires.ftc.teamcode.core.control

import kotlin.math.abs
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class TrapezoidProfileTest {

    private val dt = 0.02

    private fun simulate(
        profile: TrapezoidProfile,
        start: ProfileState,
        goal: ProfileState,
        seconds: Double,
        onStep: (ProfileState) -> Unit = {},
    ): ProfileState {
        var state = start
        var t = 0.0
        while (t < seconds) {
            state = profile.calculate(dt, state, goal)
            onStep(state)
            t += dt
        }
        return state
    }

    @Test
    fun reachesGoalAndRespectsConstraints() {
        val constraints = ProfileConstraints(maxVelocity = 0.5, maxAcceleration = 1.0)
        val profile = TrapezoidProfile(constraints)
        var maxVel = 0.0
        var lastVel = 0.0
        var maxAccel = 0.0

        // Trapezoid from 0 to 1: accel 0.5 s, cruise 1.5 s, decel 0.5 s = 2.5 s.
        val final = simulate(profile, ProfileState(0.0), ProfileState(1.0), seconds = 2.6) { s ->
            maxVel = maxOf(maxVel, abs(s.velocity))
            maxAccel = maxOf(maxAccel, abs(s.velocity - lastVel) / dt)
            lastVel = s.velocity
        }

        assertEquals(1.0, final.position, 1e-6)
        assertEquals(0.0, final.velocity, 1e-6)
        assertTrue("max velocity $maxVel", maxVel <= 0.5 + 1e-9)
        assertTrue("max acceleration $maxAccel", maxAccel <= 1.0 + 1e-6)
    }

    @Test
    fun drivesBackwardWhenGoalIsBehind() {
        val profile = TrapezoidProfile(ProfileConstraints(0.5, 1.0))
        var minVel = 0.0

        val final = simulate(profile, ProfileState(1.0), ProfileState(0.0), seconds = 2.6) { s ->
            minVel = minOf(minVel, s.velocity)
        }

        assertEquals(0.0, final.position, 1e-6)
        assertTrue("never exceeded reverse max velocity ($minVel)", minVel >= -0.5 - 1e-9)
        assertTrue("actually moved backward", minVel < -0.4)
    }

    @Test
    fun triangularProfileForShortMoves() {
        // Move too short to reach max velocity: must still arrive cleanly.
        val profile = TrapezoidProfile(ProfileConstraints(10.0, 1.0))
        var maxVel = 0.0

        val final = simulate(profile, ProfileState(0.0), ProfileState(0.25), seconds = 1.5) { s ->
            maxVel = maxOf(maxVel, s.velocity)
        }

        assertEquals(0.25, final.position, 1e-6)
        assertEquals(0.0, final.velocity, 1e-6)
        // Peak velocity for a 0.25-unit triangular move at 1 unit/s² is 0.5.
        assertTrue("peak velocity $maxVel", maxVel <= 0.5 + 1e-6)
    }

    @Test
    fun retargetingMidMotionReplansFromCurrentSetpoint() {
        val profile = TrapezoidProfile(ProfileConstraints(0.5, 1.0))
        var state = ProfileState(0.0)

        // Head toward 1.0 for a while, then change the goal to -0.5.
        repeat(25) { state = profile.calculate(dt, state, ProfileState(1.0)) }
        state = simulate(profile, state, ProfileState(-0.5), seconds = 4.0)

        assertEquals(-0.5, state.position, 1e-6)
        assertEquals(0.0, state.velocity, 1e-6)
    }

    @Test
    fun zeroOrNegativeDtIsANoOp() {
        val profile = TrapezoidProfile(ProfileConstraints(1.0, 1.0))
        val state = ProfileState(0.3, 0.2)
        assertEquals(state, profile.calculate(0.0, state, ProfileState(1.0)))
        assertEquals(state, profile.calculate(-0.01, state, ProfileState(1.0)))
    }
}
