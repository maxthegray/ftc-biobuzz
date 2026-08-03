package org.firstinspires.ftc.teamcode.core.logging

import com.qualcomm.robotcore.hardware.HardwareMap
import java.io.File
import org.firstinspires.ftc.teamcode.core.command.Commands
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class FlightRecorderRateTest {

    @Test
    fun continuousChannelsAreCappedWhileCommandsEventsAndTimingPeaksArePreserved() {
        val logDir = File.createTempFile("rate-limited-logs", "").also {
            it.delete()
            it.mkdirs()
        }
        try {
            val clock = FakeClock()
            val robot = Robot(HardwareMap(null, null), clock)
            var subsystemLogCalls = 0
            robot.register(object : SubsystemBase("Test") {
                override fun logState(log: StateLog) {
                    subsystemLogCalls++
                    log.put("calls", subsystemLogCalls.toLong())
                }
            })
            robot.enableFlightRecorder(
                "RateTest",
                driver = { null },
                operator = { null },
                batteryVoltage = { 12.5 },
                directory = logDir,
            )

            val requirement = Any()
            val first = Commands.infinite {}.requiring(requirement).setName("first")
            val second = Commands.infinite {}.requiring(requirement).setName("second")

            robot.start()
            robot.scheduler.schedule(first)
            robot.loop()

            clock.advanceMs(1.0)
            robot.scheduler.schedule(second)
            robot.loop()

            clock.advanceMs(1.0)
            robot.scheduler.cancel(second)
            robot.loop()
            robot.recordEvent("between samples")

            clock.advanceMs(5.0)
            robot.loop()

            clock.advanceMs(3.0)
            robot.loop()
            robot.stop()

            val log = WpiLog.read(logDir.listFiles { f -> f.extension == "wpilog" }!!.single())
            assertEquals(2, subsystemLogCalls)
            assertEquals(2, log.doubles("battery").size)
            assertEquals(2, log.longs("loop/totalNanos").size)
            assertEquals(5_000_000L, log.longs("loop/windowMaxTotalNanos").maxOf { it.second })
            assertEquals(
                listOf("first", "second", ""),
                log.strings("commands/running").map { it.second },
            )
            assertTrue(log.strings("events").any { it.second == "between samples" })
        } finally {
            logDir.deleteRecursively()
        }
    }
}
