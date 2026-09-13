package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
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
    private lateinit var subsystem: SubsystemBase

    @Before
    fun setUp() {
        robot = Robot(HardwareMap(null, null), clock)
        subsystem = robot.register(RecordingSubsystem(events))
    }

    @Test
    fun loopRunsPhasesInOrder() {
        val cmd = Command.build().setExecute { events += "command" }.setDone { true }

        robot.start()
        robot.loop(
            input = { events += "input" },
            control = {
                events += "control"
                Scheduler.schedule(cmd)
            },
            telemetry = { events += "telemetry" },
        )

        assertEquals(listOf("periodic", "input", "control", "command", "write", "telemetry"), events)
    }

    @Test
    fun constructingARobotClearsCommandsLeftByAPreviousOpMode() {
        val leftover = infinite {}
        Scheduler.schedule(leftover)
        Robot(HardwareMap(null, null), clock)
        assertFalse(Scheduler.isScheduled(leftover))
    }

    @Test
    fun initTickReadsButNeverWritesOrRunsCommands() {
        var executed = false
        Scheduler.schedule(infinite { executed = true })
        robot.initTick()
        assertEquals(listOf("periodic"), events)
        assertFalse(executed)
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
    fun defaultCommandIsScheduledInsideTheSchedulerPhase() {
        subsystem.defaultCommand = Command.build()
            .requiring(subsystem)
            .setStart { events += "default-start" }
            .setExecute { events += "default-execute" }

        robot.start()
        robot.loop(control = { events += "control" })

        assertEquals(listOf("periodic", "control", "default-start", "default-execute", "write"), events)
    }

    @Test
    fun priorityActionInterruptsDefaultAndDefaultResumesWhenFree() {
        var defaultStarts = 0
        subsystem.defaultCommand = Command.build().requiring(subsystem).setStart { defaultStarts++ }
        val action = Command.build()
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
    fun defaultDoesNotPreemptEqualPriorityExplicitCommand() {
        var defaultStarts = 0
        subsystem.defaultCommand = Command.build().requiring(subsystem).setStart { defaultStarts++ }
        var actionDone = false
        val actionEnds = mutableListOf<EndCondition>()
        val action = Command.build()
            .requiring(subsystem)
            .setDone { actionDone }
            .setEnd { actionEnds += it }

        robot.start()
        robot.loop()
        assertEquals(1, defaultStarts)
        // Priority 0, same as the default: explicit scheduling preempts the default...
        Scheduler.schedule(action)
        robot.loop()
        robot.loop()
        // ...but the default never steals the subsystem back.
        assertTrue(Scheduler.isScheduled(action))
        assertTrue(actionEnds.isEmpty())
        assertEquals(1, defaultStarts)

        actionDone = true
        robot.loop()
        assertEquals(listOf(EndCondition.NATURALLY), actionEnds)
        robot.loop()
        assertEquals(2, defaultStarts)
    }

    @Test
    fun defaultResumesAfterPriorityActionIsCancelled() {
        var defaultStarts = 0
        subsystem.defaultCommand = Command.build().requiring(subsystem).setStart { defaultStarts++ }
        val action = Command.build().requiring(subsystem).setPriority(CommandPriorities.DRIVER_ACTION)

        robot.start()
        robot.loop()
        Scheduler.schedule(action)
        robot.loop()
        Scheduler.cancel(action)
        robot.loop()

        assertEquals(2, defaultStarts)
    }

    @Test
    fun commandFaultAbortsEveryCommandHaltsEverySubsystemAndTheLoopContinues() {
        val other = robot.register(RecordingSubsystem(events, "other"))
        val survivorEnds = mutableListOf<EndCondition>()
        val unrelated = infinite {}.requiring(Any()).setEnd { survivorEnds += it }
        val bad = Command.build().requiring(subsystem).setExecute { error("boom") }

        robot.start()
        robot.loop(control = {
            Scheduler.schedule(unrelated)
            Scheduler.schedule(bad)
        })

        assertEquals(1, robot.commandFaultCount)
        assertEquals("boom", robot.lastCommandFault?.message)
        assertFalse(Scheduler.isScheduled(bad))
        assertFalse(Scheduler.isScheduled(unrelated))
        // No end handler runs on abort: subsystems are halted directly instead.
        assertTrue(survivorEnds.isEmpty())
        assertEquals(listOf("periodic", "periodic", "fault", "fault", "write", "write"), events)

        var defaultStarted = false
        other.defaultCommand = Command.build().requiring(other).setStart { defaultStarted = true }
        robot.loop()
        assertTrue(defaultStarted)
        assertEquals(2L, robot.loopCount)
    }

    @Test
    fun faultInTheInputPhaseAbortsCommandsToo() {
        robot.start()
        robot.loop(input = { error("binding blew up") })
        assertEquals(1, robot.commandFaultCount)
        assertEquals(listOf("periodic", "fault", "write"), events)
    }

    @Test
    fun faultsOutsideTheCommandPhasesEndTheOpMode() {
        robot.start()
        assertThrows(IllegalStateException::class.java) { robot.loop(control = { error("onLoop bug") }) }
        assertEquals(0, robot.commandFaultCount)
    }

    @Test
    fun telemetryFaultIsContainedAndCounted() {
        robot.start()
        robot.loop(telemetry = { error("dashboard hiccup") })
        assertEquals(listOf("periodic", "write"), events)
        assertEquals(1L, robot.loopCount)
        assertEquals(1, robot.telemetryFaultCount)
    }

    @Test
    fun stopResetsSchedulerWithoutRunningEndHandlersAndStopsSubsystems() {
        var ended = false
        val endless = infinite {}.setEnd { ended = true }
        Scheduler.schedule(endless)
        robot.stop()
        assertEquals(listOf("stop"), events)
        assertFalse(Scheduler.isScheduled(endless))
        assertFalse(ended)
    }

    @Test
    fun stopSwallowsSubsystemExceptionsSoAllGetCleanup() {
        robot.register(object : SubsystemBase("thrower") {
            override fun stop() {
                events += "thrower"
                error("cleanup failure")
            }
        })
        robot.register(RecordingSubsystem(events, "second"))
        robot.stop()
        assertEquals(listOf("stop", "thrower", "stop"), events)
    }

    @Test
    fun stateIsPersistedOnlyAfterALoopHasRun() {
        val persisted = mutableListOf<String>()
        robot.register(object : SubsystemBase("persist") {
            override fun persistState() { persisted += "persist" }
        })
        robot.stop()
        assertTrue(persisted.isEmpty())

        val second = Robot(HardwareMap(null, null), clock)
        second.register(object : SubsystemBase("persist") {
            override fun persistState() { persisted += "persist" }
        })
        second.start()
        second.loop()
        second.stop()
        assertEquals(listOf("persist"), persisted)
    }

    @Test
    fun registerAfterContractIsEnforced() {
        class First : SubsystemBase("first")
        class Second : SubsystemBase("second") {
            override val registerAfter: Class<out SubsystemBase> get() = First::class.java
        }

        val fresh = Robot(HardwareMap(null, null), clock)
        assertThrows(IllegalStateException::class.java) { fresh.register(Second()) }
        fresh.register(First())
        fresh.register(Second())
        assertEquals(2, fresh.subsystems().size)
    }
}
