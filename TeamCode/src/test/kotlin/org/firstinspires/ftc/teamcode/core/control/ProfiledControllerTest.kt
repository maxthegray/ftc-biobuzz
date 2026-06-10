package org.firstinspires.ftc.teamcode.core.control

import kotlin.math.abs
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ProfiledControllerTest {

    /**
     * Plant model: a perfect velocity actuator (output is the achieved
     * velocity). With kV = 1 the controller's velocity feedforward drives the
     * plant along the profile and kP trims position error.
     */
    @Test
    fun convergesOnGoalAlongProfile() {
        val controller = ProfiledController(
            ProfileConstraints(maxVelocity = 2.0, maxAcceleration = 4.0),
            PIDFGains(kP = 4.0, kV = 1.0),
        )
        controller.reset(measuredPosition = 0.0)
        controller.setGoal(10.0)

        val dt = 0.02
        var position = 0.0
        var maxSpeed = 0.0
        repeat(400) {
            val output = controller.update(dt, position)
            position += output * dt
            maxSpeed = maxOf(maxSpeed, abs(output))
        }

        assertEquals(10.0, position, 0.05)
        assertTrue(controller.atGoal(position, toleranceUnits = 0.1))
        // Velocity stayed near the profile limit (kP trim allows slight overshoot).
        assertTrue("max speed $maxSpeed", maxSpeed <= 2.0 * 1.2)
    }

    @Test
    fun atGoalRequiresProfileCompletionAndProximity() {
        val controller = ProfiledController(
            ProfileConstraints(1.0, 1.0),
            PIDFGains(kP = 1.0),
        )
        controller.reset(0.0)
        controller.setGoal(5.0)

        controller.update(0.02, 0.0)
        // Mechanism magically at the goal, but the profile has barely started.
        assertFalse(controller.atGoal(5.0, toleranceUnits = 0.1))
    }

    @Test
    fun resetSeedsProfileAtMeasuredState() {
        val controller = ProfiledController(
            ProfileConstraints(1.0, 1.0),
            PIDFGains(kP = 1.0),
        )
        controller.reset(measuredPosition = 7.5)
        assertEquals(7.5, controller.setpoint.position, 1e-9)
        assertEquals(7.5, controller.goal.position, 1e-9)
        assertTrue(controller.atGoal(7.5, toleranceUnits = 0.01))
    }
}
