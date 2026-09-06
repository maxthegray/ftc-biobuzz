package org.firstinspires.ftc.teamcode.core.runtime

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.command.Commands
import org.junit.Assert.*
import org.junit.Test
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.concurrent.thread

class RobotShutdownTest {
    @Test
    fun blockedPersistenceCannotDelayActuatorShutdown() {
        val entered = CountDownLatch(1)
        val release = CountDownLatch(1)
        val powered = AtomicBoolean(true)
        val robot = Robot(HardwareMap(null, null))
        robot.register(object : SubsystemBase("motor") {
            override fun stop() { powered.set(false) }
            override fun persistState() { entered.countDown(); release.await() }
        })
        robot.start()
        robot.loop()
        val worker = thread { robot.stop() }
        try {
            assertTrue("persistence did not run", entered.await(1, TimeUnit.SECONDS))
            assertFalse("motor still powered while storage is blocked", powered.get())
        } finally {
            release.countDown()
            worker.join(1000)
        }
        assertFalse(worker.isAlive)
    }

    @Test
    fun blockedCrashReportingSeesStoppedHardwareAndPreCleanupCommands() {
        val entered = CountDownLatch(1)
        val release = CountDownLatch(1)
        val powered = AtomicBoolean(true)
        val ended = AtomicBoolean(false)
        val robot = Robot(HardwareMap(null, null))
        robot.register(object : SubsystemBase("motor") {
            override fun stop() { powered.set(false) }
        })
        val command = Commands.infinite {}.setName("drive at crash").setEnd { ended.set(true) }
        robot.scheduler.schedule(command)
        var runningAtCrash = emptyList<String>()
        val worker = thread {
            robot.stop {
                runningAtCrash = robot.scheduler.runningCommandNames()
                entered.countDown()
                release.await()
                error("crash report storage failed")
            }
        }
        try {
            assertTrue(entered.await(1, TimeUnit.SECONDS))
            assertFalse(powered.get())
            assertEquals(listOf("drive at crash"), runningAtCrash)
            assertFalse(ended.get())
        } finally {
            release.countDown()
            worker.join(1000)
        }
        assertFalse(worker.isAlive)
        assertTrue(ended.get())
        assertTrue(robot.scheduler.runningCommands().isEmpty())
    }

    @Test
    fun everySubsystemStopsBeforeThrowingEndHandlersAndPersistence() {
        val events = mutableListOf<String>()
        val robot = Robot(HardwareMap(null, null))
        robot.register(object : SubsystemBase("first") {
            override fun stop() { events += "stop first"; error("device failed") }
            override fun persistState() { events += "persist first"; error("disk failed") }
        })
        robot.register(object : SubsystemBase("second") {
            override fun stop() { events += "stop second" }
            override fun persistState() { events += "persist second" }
        })
        robot.scheduler.schedule(Commands.infinite {}.setEnd { events += "end"; error("end failed") })
        robot.start()
        robot.loop()

        robot.stop { events += "report"; error("telemetry failed") }
        robot.stop { events += "second report" }

        assertEquals(
            listOf("stop first", "stop second", "report", "end", "persist first", "persist second"),
            events,
        )
        assertTrue(robot.scheduler.runningCommands().isEmpty())
        assertThrows(IllegalStateException::class.java) { robot.loop() }
    }
}
