package org.firstinspires.ftc.teamcode.core.runtime

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.firstinspires.ftc.teamcode.core.util.GamepadEx
import org.firstinspires.ftc.teamcode.core.util.TelemetryBag
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Test

class AutonSelectorTest {

    private class FakeSink : TelemetryBag.Sink {
        override fun addLine(text: String) {}
        override fun addData(key: String, value: String) {}
        override fun update() {}
    }

    private class SelectedRoutine(val routine: String) : RuntimeException()

    private val clock = FakeClock()
    private val robot = Robot(HardwareMap(null, null), clock)
    private val raw = Gamepad()
    private val driver = GamepadEx(raw)
    private val selector = AutonSelector(
        robot,
        TelemetryBag(listOf(FakeSink()), transmitIntervalMs = 0.0, clock = clock),
    )

    @Test
    fun allianceWritesThroughToRobotBeforeLock() {
        pressRight()

        assertEquals(Alliance.BLUE, selector.selectedAlliance)
        assertEquals(Alliance.BLUE, robot.alliance)
    }

    @Test
    fun aLocksSelectionAndIgnoresFurtherEdits() {
        selector
            .register("one") { throw SelectedRoutine("one") }
            .register("two") { throw SelectedRoutine("two") }

        pressRight()
        pressDown()
        pressRight()
        pressDown()
        pressRight()
        pressRight()
        pressA()

        assertTrue(selector.isLocked)
        assertEquals(Alliance.BLUE, selector.selectedAlliance)
        assertEquals("two", selector.selectedRoutineName)
        assertEquals(2, selector.startDelaySec)

        pressLeft()
        pressRight()
        pressUp()

        assertEquals(Alliance.BLUE, selector.selectedAlliance)
        assertEquals("two", selector.selectedRoutineName)
        assertEquals(2, selector.startDelaySec)
        assertEquals(Alliance.BLUE, robot.alliance)

        try {
            selector.selectedRunner()
            fail("Expected locked routine builder to run")
        } catch (selected: SelectedRoutine) {
            assertEquals("two", selected.routine)
        }
    }

    private fun pressA() = press { a = it }
    private fun pressUp() = press { dpad_up = it }
    private fun pressDown() = press { dpad_down = it }
    private fun pressLeft() = press { dpad_left = it }
    private fun pressRight() = press { dpad_right = it }

    private fun press(set: Gamepad.(Boolean) -> Unit) {
        raw.set(true)
        tick()
        raw.set(false)
        tick()
    }

    private fun tick() {
        driver.update(pollTriggers = false)
        selector.update(driver)
    }
}
