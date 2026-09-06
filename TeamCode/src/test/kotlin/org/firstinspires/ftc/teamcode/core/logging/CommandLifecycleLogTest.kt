package org.firstinspires.ftc.teamcode.core.logging

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.command.Commands
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import java.nio.file.Files

class CommandLifecycleLogTest {
    private val directory = Files.createTempDirectory("command-lifecycle").toFile()
    private val clock = FakeClock()
    private val robot = Robot(HardwareMap(null, null), clock).apply {
        containCommandFaults = true
        enableFlightRecorder("Lifecycle", { null }, { null }, { null }, directory)
    }

    @After
    fun cleanup() {
        robot.stop()
        directory.deleteRecursively()
    }

    private fun readLog(): WpiLog {
        robot.stop()
        return WpiLog.read(directory.listFiles { f -> f.extension == "wpilog" }!!.single())
    }

    @Test
    fun instantCommandBetweenContinuousSamplesHasBothLifecycleEvents() {
        robot.start()
        robot.loop()
        clock.advanceMs(1.0)
        var executed = false
        robot.loop(control = {
            robot.scheduler.schedule(Commands.instant { executed = true }.setName("release"))
        })

        val log = readLog()
        assertTrue(executed)
        assertTrue(log.strings("commands/running").none { "release" in it.second })
        assertEquals(
            listOf(1000L to "COMMAND STARTED: release", 1000L to "COMMAND FINISHED: release"),
            log.strings("events").filter { it.second.startsWith("COMMAND ") },
        )
    }

    @Test
    fun preemptionCancellationAndRestartKeepOriginalTimestampsWhenClosedWithoutALoop() {
        val requirement = Any()
        val first = Commands.infinite {}.requiring(requirement).setName("first")
        val second = Commands.infinite {}.requiring(requirement).setName("second")
        robot.scheduler.schedule(first)
        robot.scheduler.schedule(first) // Already running: no second start.
        clock.advanceMs(1.0)
        robot.scheduler.schedule(second)
        clock.advanceMs(1.0)
        robot.scheduler.cancel(second)
        robot.scheduler.schedule(second)
        clock.advanceMs(5.0)

        val events = readLog().strings("events").filter { it.second.startsWith("COMMAND ") }
        assertEquals(
            listOf(
                0L to "COMMAND STARTED: first",
                1000L to "COMMAND INTERRUPTED: first",
                1000L to "COMMAND STARTED: second",
                2000L to "COMMAND INTERRUPTED: second",
                2000L to "COMMAND STARTED: second",
                7000L to "COMMAND INTERRUPTED: second",
            ),
            events,
        )
    }

    @Test
    fun failuresInEveryLifecyclePhaseProduceExactlyOneFaultedEnd() {
        for (phase in listOf("start", "execute", "done", "end")) {
            val command = Command.build().setName(phase)
                .setStart { if (phase == "start") error("start failed") }
                .setExecute { if (phase == "execute") error("execute failed") }
                .setDone { if (phase == "done") error("done failed"); true }
                .setEnd { if (phase == "end") error("end failed") }
            robot.scheduler.schedule(command)
            robot.scheduler.execute()
        }

        val events = readLog().strings("events").map { it.second }
        for (phase in listOf("start", "execute", "done", "end")) {
            assertEquals(1, events.count { it == "COMMAND STARTED: $phase" })
            assertEquals(1, events.count { it == "COMMAND FAULTED: $phase" })
            assertFalse(events.contains("COMMAND FINISHED: $phase"))
        }
        assertEquals(4, robot.commandFaultCount)
    }
}
