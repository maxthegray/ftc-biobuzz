package org.firstinspires.ftc.teamcode.core.logging

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.HardwareMap
import java.io.File
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class FlightRecorderRateTest {

    @Test
    fun continuousChannelsAreCappedWhileEventsAndTimingPeaksArePreserved() {
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
            val first = infinite {}.requiring(requirement)
            val second = infinite {}.requiring(requirement)

            robot.start()
            Scheduler.schedule(first)
            robot.loop()

            clock.advanceMs(1.0)
            Scheduler.schedule(second)
            robot.loop()

            clock.advanceMs(1.0)
            Scheduler.cancel(second)
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
            // Ivy exposes no command registry; the old commands/running channel is gone.
            assertFalse(log.has("commands/running"))
            assertTrue(log.strings("events").any { it.second == "between samples" })
        } finally {
            logDir.deleteRecursively()
        }
    }

    @Test
    fun sameMicrosecondEventsStayDistinctAndLaterEventsKeepTheirOwnTime() {
        val logDir = File.createTempFile("event-timestamps", "").also {
            it.delete()
            it.mkdirs()
        }
        try {
            val clock = FakeClock()
            val robot = Robot(HardwareMap(null, null), clock)
            robot.enableFlightRecorder("EventTest", { null }, { null }, { null }, directory = logDir)
            clock.advanceMs(5.0)
            robot.recordEvent("first")
            robot.recordEvent("second")
            clock.advanceMs(5.0)
            robot.stopAfterCrash(IllegalStateException("boom"))

            val events = WpiLog.read(logDir.listFiles { f -> f.extension == "wpilog" }!!.single()).strings("events")
            assertEquals(
                listOf(0L to "init EventTest", 5_000L to "first", 5_001L to "second", 10_000L to "LOOP CRASHED", 10_001L to "stop"),
                events.map { (ts, text) -> ts to text.substringBefore(":") },
            )
        } finally {
            logDir.deleteRecursively()
        }
    }
}
