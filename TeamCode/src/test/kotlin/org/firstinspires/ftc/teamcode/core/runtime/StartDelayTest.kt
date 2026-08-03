package org.firstinspires.ftc.teamcode.core.runtime

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.sim.FakeSink
import org.firstinspires.ftc.teamcode.core.util.GamepadEx
import org.firstinspires.ftc.teamcode.core.util.TelemetryBag
import org.junit.Assert.assertEquals
import org.junit.Test

class StartDelayTest {

    private val clock = FakeClock()
    private val robot = Robot(HardwareMap(null, null), clock)
    private val raw = Gamepad()
    private val driver = GamepadEx(raw, robot.scheduler)
    private val sink = FakeSink()
    private val bag = TelemetryBag(listOf(sink), transmitIntervalMs = 0.0, clock = clock)

    @Test
    fun rightIncrementsAndLeftDecrements() {
        val delay = StartDelay(bag)

        pressRight(delay)
        pressRight(delay)
        assertEquals(2, delay.seconds)

        pressLeft(delay)
        assertEquals(1, delay.seconds)
    }

    @Test
    fun clampsAtZeroAndMax() {
        val delay = StartDelay(bag, maxSec = 2)

        repeat(4) { pressRight(delay) }
        assertEquals(2, delay.seconds)

        repeat(4) { pressLeft(delay) }
        assertEquals(0, delay.seconds)
    }

    @Test
    fun millisTracksSeconds() {
        val delay = StartDelay(bag)

        assertEquals(0L, delay.millis)
        pressRight(delay)
        pressRight(delay)
        pressRight(delay)
        assertEquals(3000L, delay.millis)
    }

    @Test
    fun publishesSelectedDelayToTelemetry() {
        val delay = StartDelay(bag)

        pressRight(delay)
        bag.flush()

        assertEquals("1", sink.latest("start delay s"))
    }

    private fun pressLeft(delay: StartDelay) = press(delay) { dpad_left = it }
    private fun pressRight(delay: StartDelay) = press(delay) { dpad_right = it }

    private fun press(delay: StartDelay, set: Gamepad.(Boolean) -> Unit) {
        raw.set(true)
        tick(delay)
        raw.set(false)
        tick(delay)
    }

    private fun tick(delay: StartDelay) {
        driver.update(pollTriggers = false)
        delay.update(driver)
    }
}
