package org.firstinspires.ftc.teamcode.core.logging

import com.qualcomm.robotcore.hardware.HardwareMap
import java.io.File
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class FlightRecorderFaultTest {

    private lateinit var logDir: File

    @Before
    fun setUp() {
        logDir = File.createTempFile("flight-recorder-faults", "").also {
            it.delete()
            it.mkdirs()
        }
    }

    @After
    fun tearDown() {
        logDir.deleteRecursively()
    }

    @Test
    fun openCreatesUniqueFilesWithinTheSameTimestamp() {
        val first = openRecorder("TeleOp")
        val second = openRecorder("TeleOp")

        first.close()
        second.close()

        val files = logDir.listFiles { file -> file.extension == "wpilog" }.orEmpty()
        assertEquals(2, files.size)
        assertEquals(2, files.map { it.name }.distinct().size)
    }

    @Test
    fun throwingSubsystemLoggingIsDisabledWithoutStoppingOtherLogging() {
        val clock = FakeClock()
        val robot = Robot(HardwareMap(null, null), clock)
        var badCalls = 0
        var goodCalls = 0
        robot.register(object : SubsystemBase("Bad") {
            override fun logState(log: StateLog) {
                badCalls++
                error("student bug")
            }
        })
        robot.register(object : SubsystemBase("Good") {
            override fun logState(log: StateLog) {
                goodCalls++
                log.put("calls", goodCalls.toLong())
            }
        })
        robot.enableFlightRecorder(
            "TeleOp",
            driver = { null },
            operator = { null },
            batteryVoltage = { 12.5 },
            directory = logDir,
        )

        robot.start()
        robot.loop()
        clock.advanceMs(20.0)
        robot.loop()
        robot.stop()

        assertEquals(1, badCalls)
        assertEquals(2, goodCalls)
        val log = WpiLog.read(logDir.listFiles()!!.single())
        assertEquals(listOf(1L, 2L), log.longs("Good/calls").map { it.second })
        assertTrue(
            log.strings("events").any {
                "SUBSYSTEM LOGGING DISABLED: Bad: IllegalStateException: student bug" in it.second
            },
        )
    }

    @Test
    fun escapedRecorderFaultDisablesOnlyTheRecorder() {
        val robot = Robot(HardwareMap(null, null), FakeClock())
        robot.enableFlightRecorder(
            "TeleOp",
            driver = { null },
            operator = { null },
            batteryVoltage = { error("voltage supplier failed") },
            directory = logDir,
        )

        robot.start()
        robot.loop()
        robot.loop()

        assertEquals(2L, robot.loopCount)
        assertEquals(1, robot.recorderFaultCount)
        assertTrue(robot.recentEvents().any { "RECORDER FAULT" in it })
    }

    private fun openRecorder(name: String): FlightRecorder =
        checkNotNull(
            FlightRecorder.open(
                name,
                gamepad1 = { null },
                gamepad2 = { null },
                batteryVoltage = { null },
                runningCommandNames = { emptyList() },
                directory = logDir,
            ),
        )
}
