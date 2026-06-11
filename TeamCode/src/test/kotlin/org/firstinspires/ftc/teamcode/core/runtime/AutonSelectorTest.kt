package org.firstinspires.ftc.teamcode.core.runtime

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import java.io.File
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
    private val driver = GamepadEx(raw, robot.scheduler)
    private val bag = TelemetryBag(listOf(FakeSink()), transmitIntervalMs = 0.0, clock = clock)
    private val selector by lazy { AutonSelector(robot, bag) }

    private lateinit var tempFile: File
    private var originalFile: File? = null

    @org.junit.Before
    fun setUp() {
        tempFile = File.createTempFile("auton-selection", ".txt").also { it.delete() }
        originalFile = AutonSelector.storageFile
        AutonSelector.storageFile = tempFile
    }

    @org.junit.After
    fun tearDown() {
        AutonSelector.storageFile = originalFile
        tempFile.delete()
    }

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

    @Test
    fun lockedSelectionPersistsAndRestoresAsUnlockedDefault() {
        selector
            .register("one") { throw SelectedRoutine("one") }
            .register("two") { throw SelectedRoutine("two") }

        pressRight() // BLUE
        pressDown()
        pressRight() // routine "two"
        pressDown()
        pressRight() // delay 1
        pressA()
        assertTrue(tempFile.exists())

        // "Re-init": fresh selector restores the locked choices as defaults…
        val restored = AutonSelector(robot, bag)
            .register("one") { throw SelectedRoutine("one") }
            .register("two") { throw SelectedRoutine("two") }
        assertEquals(Alliance.BLUE, restored.selectedAlliance)
        assertEquals("two", restored.selectedRoutineName)
        assertEquals(1, restored.startDelaySec)
        // …but unlocked: A still has to confirm.
        assertTrue(!restored.isLocked)
    }

    @Test
    fun restoredRoutineIndexClampsToRegisteredRoutines() {
        tempFile.writeText("auton-v1 RED 5 0")
        val restored = AutonSelector(robot, bag)
            .register("only") { throw SelectedRoutine("only") }
        assertEquals("only", restored.selectedRoutineName)
    }

    @Test
    fun corruptSelectionFileIsIgnored() {
        tempFile.writeText("garbage")
        val restored = AutonSelector(robot, bag)
            .register("only") { throw SelectedRoutine("only") }
        assertEquals(Alliance.RED, restored.selectedAlliance)
        assertEquals(0, restored.startDelaySec)
    }
}
