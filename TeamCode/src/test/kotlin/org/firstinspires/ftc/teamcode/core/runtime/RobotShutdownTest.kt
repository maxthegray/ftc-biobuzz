package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.concurrent.thread
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

class RobotShutdownTest {
    @Test
    fun blockedCrashReportingSeesStoppedHardware() {
        val entered = CountDownLatch(1)
        val release = CountDownLatch(1)
        val powered = AtomicBoolean(true)
        val robot = Robot(HardwareMap(null, null))
        robot.register(object : SubsystemBase("motor") {
            override fun stop() { powered.set(false) }
        })
        val worker = thread {
            robot.stop {
                entered.countDown()
                release.await()
                error("crash report storage failed")
            }
        }
        try {
            assertTrue(entered.await(1, TimeUnit.SECONDS))
            assertFalse(powered.get())
        } finally {
            release.countDown()
            worker.join(1000)
        }
        assertFalse(worker.isAlive)
    }

    @Test
    fun endHandlersCannotReenergizeStoppedHardware() {
        val events = mutableListOf<String>()
        var power = 0.0
        val robot = Robot(HardwareMap(null, null))
        robot.register(object : SubsystemBase("first") {
            override fun stop() { power = 0.0; events += "stop first"; error("device failed") }
        })
        robot.register(object : SubsystemBase("second") {
            override fun stop() { events += "stop second" }
        })
        val running = infinite { power = 1.0 }.setEnd { power = 1.0; events += "end" }
        Scheduler.schedule(running)
        robot.start()
        robot.loop()
        assertEquals(1.0, power, 0.0)

        robot.stop { events += "report"; error("telemetry failed") }
        robot.stop { events += "second report" }

        assertEquals(listOf("stop first", "stop second", "report"), events)
        assertEquals(0.0, power, 0.0)
        assertFalse(Scheduler.isScheduled(running))
        assertThrows(IllegalStateException::class.java) { robot.loop() }
    }
}
