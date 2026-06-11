package org.firstinspires.ftc.teamcode.core.runtime

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.command.CommandBuilder
import org.firstinspires.ftc.teamcode.core.util.FakeClock
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
        override fun onCommandFault() { events += "fault" }
        override fun stop() { events += "stop" }
    }

    private val events = mutableListOf<String>()
    private val clock = FakeClock()
    private lateinit var robot: Robot

    @Before
    fun setUp() {
        robot = Robot(HardwareMap(null, null), clock)
        robot.register(RecordingSubsystem(events))
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
                robot.scheduler.schedule(cmd)
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
        robot.scheduler.schedule(endless)
        robot.stop()
        assertEquals(listOf("stop"), events)
        assertTrue(!robot.scheduler.isScheduled(endless))
    }

    @Test
    fun stopRunsEndHandlersOfRunningCommands() {
        var ended: org.firstinspires.ftc.teamcode.core.command.EndCondition? = null
        val endless = CommandBuilder()
            .setDone { false }
            .setEnd { ended = it }
        robot.scheduler.schedule(endless)
        robot.stop()
        assertEquals(org.firstinspires.ftc.teamcode.core.command.EndCondition.INTERRUPTED, ended)
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
        robot.scheduler.schedule(action)
        robot.loop()
        robot.loop()

        assertEquals(2, defaultStarts)
    }

    @Test
    fun uncontainedCommandFaultPropagates() {
        val bad = CommandBuilder()
            .setExecute { error("boom") }
            .setDone { false }

        robot.start()
        try {
            robot.loop(control = { robot.scheduler.schedule(bad) })
            org.junit.Assert.fail("expected the command fault to propagate")
        } catch (e: IllegalStateException) {
            assertEquals("boom", e.message)
        }
        assertEquals(0, robot.commandFaultCount)
    }

    @Test
    fun containedCommandFaultIsSurgical() {
        robot.containCommandFaults = true
        val faultedSubsystem = robot.subsystems().first()
        // A healthy command on an unrelated requirement must survive the fault.
        val survivorExecutes = mutableListOf<Long>()
        val survivor = CommandBuilder()
            .requiring(Any())
            .setExecute { survivorExecutes += robot.loopCount }
            .setDone { false }
        val bad = CommandBuilder()
            .requiring(faultedSubsystem)
            .setExecute { error("boom") }
            .setDone { false }

        robot.start()
        robot.loop(control = {
            robot.scheduler.schedule(survivor)
            robot.scheduler.schedule(bad)
        })

        assertEquals(1, robot.commandFaultCount)
        assertEquals("boom", robot.lastCommandFault?.message)
        assertTrue(!robot.scheduler.isScheduled(bad))
        // The survivor is still running; the faulted command's subsystem got
        // its onCommandFault hook; the loop completed its write phase.
        assertTrue(robot.scheduler.isScheduled(survivor))
        assertEquals(listOf("periodic", "fault", "write"), events)

        // Default commands resume on the next tick.
        var defaultStarted = false
        faultedSubsystem.defaultCommand = CommandBuilder()
            .requiring(faultedSubsystem)
            .setStart { defaultStarted = true }
            .setDone { false }
        robot.loop()
        assertTrue(defaultStarted)
        assertTrue(robot.scheduler.isScheduled(survivor))
    }

    @Test
    fun containedFaultWithoutRequirementsTouchesNoSubsystems() {
        robot.containCommandFaults = true
        val bad = CommandBuilder()
            .setExecute { error("boom") }
            .setDone { false }

        robot.start()
        robot.loop(control = { robot.scheduler.schedule(bad) })

        assertEquals(1, robot.commandFaultCount)
        // No requirements -> no onCommandFault calls.
        assertEquals(listOf("periodic", "write"), events)
    }

    @Test
    fun faultedCommandEndHandlerRunsBeforeContainment() {
        robot.containCommandFaults = true
        var ended: org.firstinspires.ftc.teamcode.core.command.EndCondition? = null
        val bad = CommandBuilder()
            .setExecute { error("boom") }
            .setEnd { ended = it }
            .setDone { false }

        robot.start()
        robot.loop(control = { robot.scheduler.schedule(bad) })

        assertEquals(org.firstinspires.ftc.teamcode.core.command.EndCondition.FAULTED, ended)
        assertEquals(1, robot.commandFaultCount)
    }

    @Test
    fun containedFaultInTriggerPollingKeepsLooping() {
        robot.containCommandFaults = true

        robot.start()
        robot.loop(input = { error("binding blew up") })

        // Not tied to a command: counted and logged, no subsystem safing.
        assertEquals(1, robot.commandFaultCount)
        assertEquals(listOf("periodic", "write"), events)
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
        robot.scheduler.schedule(action)
        robot.loop()
        robot.scheduler.cancel(action)
        robot.loop()

        assertEquals(2, defaultStarts)
    }

    @Test
    fun registerAfterContractIsEnforced() {
        class First : SubsystemBase("first")
        class Second : SubsystemBase("second") {
            override val registerAfter: Class<out SubsystemBase> get() = First::class.java
        }

        val fresh = Robot(HardwareMap(null, null), clock)
        try {
            fresh.register(Second())
            org.junit.Assert.fail("expected registration-order check to throw")
        } catch (_: IllegalStateException) {
            // expected
        }

        fresh.register(First())
        fresh.register(Second())
        assertEquals(2, fresh.subsystems().size)
    }
}
