package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class RobotLoopTest {

    private class RecordingSubsystem(
        private val events: MutableList<String>,
        name: String = "rec",
    ) : SubsystemBase(name) {
        override fun periodic() { events += "periodic" }
        override fun writeHardware() { events += "write" }
        override fun stop() { events += "stop" }
    }

    private val events = mutableListOf<String>()
    private val clock = FakeClock()
    private lateinit var robot: Robot

    @Before
    fun setUp() {
        Scheduler.reset()
        robot = Robot(HardwareMap(null, null), clock)
        robot.register(RecordingSubsystem(events))
    }

    @After
    fun tearDown() {
        Scheduler.reset()
    }

    @Test
    fun loopRunsPhasesInOrder() {
        // A command scheduled during control whose execute records proves the
        // scheduler phase sits between control and writeHardware.
        val cmd = CommandBuilder()
            .setExecute { events += "command" }
            .setDone { true }

        robot.start()
        robot.loop(
            input = { events += "input" },
            control = {
                events += "control"
                Scheduler.schedule(cmd)
            },
            telemetry = { events += "telemetry" },
        )

        assertEquals(
            listOf("periodic", "input", "control", "command", "write", "telemetry"),
            events,
        )
    }

    @Test
    fun initTickReadsButNeverWrites() {
        robot.initTick()
        assertEquals(listOf("periodic"), events)
    }

    @Test
    fun loopCountAndDurationAdvance() {
        robot.start()
        clock.advanceMs(5.0)
        robot.loop()
        assertEquals(1, robot.loopCount)
        assertTrue(robot.lastLoopNanos > 0)
        assertTrue(robot.loopHz > 0.0)
    }

    @Test
    fun stopResetsSchedulerAndStopsSubsystems() {
        val endless = CommandBuilder().setDone { false }
        Scheduler.schedule(endless)
        robot.stop()
        assertEquals(listOf("stop"), events)
        assertTrue(!Scheduler.isScheduled(endless))
    }

    @Test
    fun stopSwallowsSubsystemExceptionsSoAllGetCleanup() {
        val second = RecordingSubsystem(events, "second")
        robot.register(object : SubsystemBase("thrower") {
            override fun stop() {
                events += "thrower"
                error("cleanup failure")
            }
        })
        robot.register(second)
        robot.stop()
        assertEquals(listOf("stop", "thrower", "stop"), events)
    }

    @Test
    fun stopBeforeFirstLoopDoesNotPersistSubsystemState() {
        val persisted = mutableListOf<String>()
        robot.register(object : SubsystemBase("persist") {
            override fun persistState() { persisted += "persist" }
        })

        robot.stop()

        assertTrue(persisted.isEmpty())
    }

    @Test
    fun stopAfterLoopPersistsSubsystemState() {
        val persisted = mutableListOf<String>()
        robot.register(object : SubsystemBase("persist") {
            override fun persistState() { persisted += "persist" }
        })

        robot.start()
        robot.loop()
        robot.stop()

        assertEquals(listOf("persist"), persisted)
    }

    @Test
    fun defaultCommandRunsInsideSchedulerPhase() {
        val requirement = robot.subsystems().first()
        val default = CommandBuilder()
            .requiring(requirement)
            .setStart { events += "default-start" }
            .setExecute { events += "default-execute" }
            .setDone { false }
        requirement.defaultCommand = default

        robot.start()
        robot.loop(control = { events += "control" })

        assertEquals(
            listOf("periodic", "control", "default-start", "default-execute", "write"),
            events,
        )
    }

    @Test
    fun priorityActionInterruptsDefaultAndDefaultResumesWhenFree() {
        val subsystem = robot.subsystems().first()
        var defaultStarts = 0
        val default = CommandBuilder()
            .requiring(subsystem)
            .setStart { defaultStarts++ }
            .setDone { false }
        subsystem.defaultCommand = default
        val action = CommandBuilder()
            .requiring(subsystem)
            .setPriority(CommandPriorities.DRIVER_ACTION)
            .setDone { true }

        robot.start()
        robot.loop()
        Scheduler.schedule(action)
        robot.loop()
        robot.loop()

        assertEquals(2, defaultStarts)
    }

    @Test
    fun defaultResumesAfterPriorityActionIsCancelled() {
        val subsystem = robot.subsystems().first()
        var defaultStarts = 0
        val default = CommandBuilder()
            .requiring(subsystem)
            .setStart { defaultStarts++ }
            .setDone { false }
        subsystem.defaultCommand = default
        val action = CommandBuilder()
            .requiring(subsystem)
            .setPriority(CommandPriorities.DRIVER_ACTION)
            .setDone { false }

        robot.start()
        robot.loop()
        Scheduler.schedule(action)
        robot.loop()
        Scheduler.cancel(action)
        robot.loop()

        assertEquals(2, defaultStarts)
    }
}
