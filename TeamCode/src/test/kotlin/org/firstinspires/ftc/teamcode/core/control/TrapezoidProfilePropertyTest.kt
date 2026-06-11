package org.firstinspires.ftc.teamcode.core.control

import kotlin.math.abs
import kotlin.random.Random
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Profile invariants under randomized mid-motion goal changes — the
 * scenarios a season actually produces (driver re-targets a lift mid-travel)
 * and exactly where hand-written cases miss edge geometry.
 */
class TrapezoidProfilePropertyTest {

    private companion object {
        const val SCENARIOS = 100
        const val STEPS = 300
        const val DT = 0.02
        // Velocity may legitimately exceed maxVelocity for one step when a
        // goal flips behind a fast-moving setpoint (the profile decelerates
        // at maxAcceleration, it cannot clamp instantaneously) — the real
        // invariants are the acceleration bound and continuity.
        const val EPS = 1e-9
    }

    @Test
    fun accelerationAndContinuityHoldUnderRandomGoalChanges() {
        val random = Random(20260611)
        repeat(SCENARIOS) { scenario ->
            val maxVelocity = random.nextDouble(5.0, 80.0)
            val maxAcceleration = random.nextDouble(10.0, 400.0)
            val profile = TrapezoidProfile(ProfileConstraints(maxVelocity, maxAcceleration))

            var setpoint = ProfileState(random.nextDouble(-50.0, 50.0))
            var goal = ProfileState(random.nextDouble(-50.0, 50.0))

            repeat(STEPS) { step ->
                if (random.nextDouble() < 0.05) {
                    goal = ProfileState(random.nextDouble(-50.0, 50.0))
                }
                val previous = setpoint
                setpoint = profile.calculate(DT, previous, goal)

                val label = "scenario $scenario step $step (vmax=$maxVelocity amax=$maxAcceleration)"

                // Acceleration bound: |Δv| ≤ amax·dt.
                assertTrue(
                    "$label: accel ${abs(setpoint.velocity - previous.velocity) / DT}",
                    abs(setpoint.velocity - previous.velocity) <= maxAcceleration * DT + EPS,
                )
                // Velocity bound (allowing the documented overspeed clamp path).
                assertTrue(
                    "$label: velocity ${setpoint.velocity}",
                    abs(setpoint.velocity) <= maxVelocity + maxAcceleration * DT + EPS,
                )
                // Position continuity: the step is bounded by the velocities
                // bracketing it.
                val bound = (maxOf(abs(previous.velocity), abs(setpoint.velocity)) + maxAcceleration * DT) * DT
                assertTrue(
                    "$label: position step ${abs(setpoint.position - previous.position)} > $bound",
                    abs(setpoint.position - previous.position) <= bound + EPS,
                )
            }
        }
    }

    @Test
    fun alwaysReachesAStationaryGoalAndStays() {
        val random = Random(7)
        repeat(SCENARIOS) {
            val maxVelocity = random.nextDouble(5.0, 80.0)
            val maxAcceleration = random.nextDouble(10.0, 400.0)
            val profile = TrapezoidProfile(ProfileConstraints(maxVelocity, maxAcceleration))
            var setpoint = ProfileState(random.nextDouble(-50.0, 50.0))
            val goal = ProfileState(random.nextDouble(-50.0, 50.0))

            // Generous budget: worst case distance 100 at min vmax 5.
            var steps = 0
            while (steps < 3000 && (setpoint.position != goal.position || setpoint.velocity != goal.velocity)) {
                setpoint = profile.calculate(DT, setpoint, goal)
                steps++
            }
            assertTrue("did not converge (vmax=$maxVelocity amax=$maxAcceleration)", steps < 3000)

            // And it stays put.
            val settled = profile.calculate(DT, setpoint, goal)
            assertTrue(abs(settled.position - goal.position) < EPS)
            assertTrue(abs(settled.velocity) < EPS)
        }
    }
}
