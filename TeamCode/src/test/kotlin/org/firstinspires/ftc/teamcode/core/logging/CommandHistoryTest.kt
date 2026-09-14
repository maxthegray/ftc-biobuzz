package org.firstinspires.ftc.teamcode.core.logging

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.behaviors.BlockedBehavior
import com.pedropathing.ivy.behaviors.ConflictBehavior
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.behaviors.InterruptedBehavior
import com.pedropathing.ivy.commands.Commands.infinite
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.lazy
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.parallel
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.repeat
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.hardware.HardwareMap
import java.io.File
import java.io.IOException
import java.io.UncheckedIOException
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertSame
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/** [logged] and [CommandHistory] against Ivy 1.1.1, read back from the WPILOG. */
class CommandHistoryTest {

    private lateinit var logDir: File
    private val clock = FakeClock()
    private lateinit var robot: Robot
    private val arm = object : SubsystemBase("Arm") {
        var power = 0.0
        var stopCalls = 0
        override fun onCommandFault() { power = 0.0 }
        override fun stop() {
            power = 0.0
            stopCalls++
            // Anything recorded after hardware stops is visibly later.
            clock.advanceMs(5.0)
        }
    }

    @Before
    fun setUp() {
        logDir = File.createTempFile("command-history", "").also { it.delete(); it.mkdirs() }
        robot = Robot(HardwareMap(null, null), clock)
        robot.register(arm)
        robot.enableFlightRecorder("History", { null }, { null }, { null }, directory = logDir)
        robot.init()
        robot.start()
    }

    @After
    fun tearDown() {
        robot.stop()
        logDir.deleteRecursively()
    }

    private fun tick(ms: Double = 20.0, input: () -> Unit = {}) {
        clock.advanceMs(ms)
        robot.loop(input = input)
    }

    private fun log(): WpiLog {
        robot.stop()
        return WpiLog.read(logDir.listFiles { f -> f.extension == "wpilog" }!!.single())
    }

    private fun WpiLog.commandEvents() = strings("commands/events").map { it.second }
    private fun WpiLog.active() = strings("commands/active").map { it.second }

    private fun armCommand(done: () -> Boolean = { false }) = Command.build().requiring(arm).setDone(done)

    @Test
    fun naturalCompletionPreemptionAndCancellation() {
        var finished = false
        val a = logged("Raise", armCommand { finished })
        Scheduler.schedule(a)
        tick()
        finished = true
        tick()
        val b = logged("Hold", armCommand())
        Scheduler.schedule(b)
        tick()
        val c = logged("Override", armCommand().setPriority(CommandPriorities.DRIVER_OVERRIDE))
        Scheduler.schedule(c)
        tick()
        Scheduler.cancel(c)

        val log = log()
        assertEquals(
            listOf("START #1 Raise", "FINISH #1 Raise", "START #2 Hold", "INTERRUPT #2 Hold", "START #3 Override", "INTERRUPT #3 Override"),
            log.commandEvents(),
        )
        assertEquals(listOf("", "#1 Raise", "", "#2 Hold", "", "#3 Override", ""), log.active())
        val times = log.strings("commands/events").map { it.first }
        assertTrue(times.zipWithNext().all { (x, y) -> y > x })
        // Scheduled before the first tick: after the initial records at 0 µs, not on top of them.
        assertEquals(1L, times[0])
        assertEquals(40_000L, times[1])
        val active = log.strings("commands/active").map { it.first }
        assertTrue(active.zipWithNext().all { (x, y) -> y > x })
        assertEquals(0L, log.longs("commands/lost").single().second)
    }

    @Test
    fun wrappingForwardsPoliciesResultsAndExceptionsUnchanged() {
        val boom = IllegalStateException("boom")
        val inner = armCommand { true }
            .setPriority(7)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            .setConflictBehavior(ConflictBehavior.QUEUE)
            .setExecute { throw boom }
        val wrapped = logged("Policy", inner)
        assertSame(inner.requirements(), wrapped.requirements())
        assertEquals(7, wrapped.priority())
        assertEquals(InterruptedBehavior.SUSPEND, wrapped.interruptedBehavior())
        assertEquals(BlockedBehavior.QUEUE, wrapped.blockedBehavior())
        assertEquals(ConflictBehavior.QUEUE, wrapped.conflictBehavior())
        assertTrue(wrapped.done())
        assertSame(boom, assertThrows(IllegalStateException::class.java) { wrapped.execute() })
        // Wrapping twice renames instead of tracing twice.
        val renamed = logged("Renamed", wrapped) as LoggedCommand
        assertSame(inner, renamed.command)
    }

    @Test
    fun failuresInEachPhaseAreRecordedOnceAndUnrelatedCommandsAreAbortedNotFailed() {
        val bystander = logged("Bystander", infinite {})
        Scheduler.schedule(bystander)
        tick()

        val phases = listOf(
            "start" to armCommand().setStart { error("start boom") },
            "execute" to armCommand().setExecute { error("execute boom") },
            "done" to armCommand().setDone { error("done boom") },
        )
        for ((phase, inner) in phases) {
            tick(input = { Scheduler.schedule(logged("Bad $phase", inner)) })
            if (phase != "start") tick()
            tick(input = { Scheduler.schedule(bystander) })
        }
        val badEnd = logged("Bad end", armCommand().setEnd { error("end boom") })
        tick(input = { Scheduler.schedule(badEnd) })
        tick(input = { Scheduler.cancel(badEnd) })

        assertEquals(4, robot.commandFaultCount)
        assertEquals("end boom", robot.lastCommandFault?.message)
        val events = log().commandEvents()
        assertEquals(
            listOf(
                "FAIL #2 Bad start in start: IllegalStateException: start boom",
                "FAIL #4 Bad execute in execute: IllegalStateException: execute boom",
                "FAIL #6 Bad done in done: IllegalStateException: done boom",
                "FAIL #8 Bad end in end: IllegalStateException: end boom",
            ),
            events.filter { it.startsWith("FAIL") },
        )
        // The bystander is closed as a consequence of each fault, never as a failure.
        assertEquals(
            listOf("ABORT #1 Bystander: command fault", "ABORT #3 Bystander: command fault", "ABORT #5 Bystander: command fault", "ABORT #7 Bystander: command fault"),
            events.filter { it.contains("Bystander") && !it.startsWith("START") },
        )
        assertEquals(4, events.count { it.startsWith("FAIL") })
    }

    @Test
    fun aFailingChildIsTheOriginAndItsTracedParentsSayWhereItCameFrom() {
        val routine = logged("Routine", sequential(logged("Step", armCommand().setExecute { error("jammed") })))
        Scheduler.schedule(routine)
        tick()
        assertEquals(
            listOf(
                "START #1 Routine",
                "START #2 Step",
                "FAIL #2 Step in execute: IllegalStateException: jammed",
                "FAIL #1 Routine in execute: IllegalStateException: jammed (from #2)",
            ),
            log().commandEvents(),
        )
    }

    @Test
    fun cleanupWithoutStartAndRepeatedCleanupRecordNothing() {
        val later = logged("Later step", armCommand())
        val group = sequential(logged("First step", armCommand()), later)
        Scheduler.schedule(group)
        tick()
        Scheduler.cancel(group)
        later.end(EndCondition.INTERRUPTED)

        var deadlineDone = false
        Scheduler.schedule(deadline(waitUntil { deadlineDone }, logged("Deadline child", infinite {})))
        tick()
        deadlineDone = true
        repeat(2) { tick() }

        Scheduler.schedule(infinite {}.requiring(arm).setPriority(5))
        val queued = logged("Queued", armCommand().setBlockedBehavior(BlockedBehavior.QUEUE))
        Scheduler.schedule(queued)
        Scheduler.cancel(queued)

        val stale = lazy { logged("Lazy inner", infinite {}) }
        Scheduler.schedule(stale)
        Scheduler.cancel(stale)
        stale.end(EndCondition.INTERRUPTED)

        assertEquals(
            listOf(
                "START #1 First step", "INTERRUPT #1 First step",
                "START #2 Deadline child", "INTERRUPT #2 Deadline child",
                "START #3 Lazy inner", "INTERRUPT #3 Lazy inner",
            ),
            log().commandEvents(),
        )
    }

    @Test
    fun nestedRepeatedAndConcurrentIdenticallyNamedCommandsKeepTheirOwnExecutions() {
        val routine = logged("Routine", sequential(logged("Step", instant {}), logged("Step", instant {})))
        Scheduler.schedule(routine)
        repeat(4) { tick() }
        val pulses = repeat(logged("Pulse", instant {}), 3)
        Scheduler.schedule(pulses)
        repeat(4) { tick() }
        val twins = parallel(logged("Twin", infinite {}), logged("Twin", infinite {}))
        Scheduler.schedule(twins)
        tick()
        Scheduler.cancel(twins)

        val log = log()
        assertEquals(
            listOf(
                "START #1 Routine", "START #2 Step", "FINISH #2 Step", "START #3 Step", "FINISH #3 Step", "FINISH #1 Routine",
                "START #4 Pulse", "FINISH #4 Pulse", "START #5 Pulse", "FINISH #5 Pulse", "START #6 Pulse", "FINISH #6 Pulse",
            ),
            log.commandEvents().take(12),
        )
        assertTrue("#7 Twin\n#8 Twin" in log.active())
        assertEquals(listOf("INTERRUPT #7 Twin", "INTERRUPT #8 Twin").toSet(), log.commandEvents().takeLast(2).toSet())
        assertEquals("", log.active().last())
    }

    @Test
    fun groupsBehaveExactlyAsTheyDoUnwrapped() {
        fun script(wrap: (String, Command) -> Command): List<String> {
            Scheduler.reset()
            val calls = mutableListOf<String>()
            fun probe(id: String, doneAfter: Int): Command {
                var ticks = 0
                return wrap(
                    id,
                    Command.build()
                        .setStart { ticks = 0; calls += "$id start" }
                        .setExecute { ticks++; calls += "$id execute" }
                        .setDone { ticks >= doneAfter }
                        .setEnd { calls += "$id end $it" },
                )
            }
            // Single-child race/parallel: with more children Ivy iterates a HashMap,
            // whose order differs from run to run with or without wrapping.
            val group = sequential(
                probe("a", 1),
                deadline(probe("b", 2), probe("c", 5)),
                race(probe("d", 1)),
                repeat(probe("e", 1), 2),
                parallel(probe("f", 3)),
                probe("g", 1),
                probe("h", 1),
            )
            val top = wrap("group", group)
            Scheduler.schedule(top)
            repeat(12) { Scheduler.execute() }
            Scheduler.cancel(top)
            return calls
        }
        assertEquals(script { _, c -> c }, script(::logged))
    }

    @Test
    fun defaultsPreemptionAndCancellationUseTheWrappedInstance() {
        val builder = Command.build().requiring(arm).setExecute { arm.power = 0.1 }
        val idle = logged("Arm idle", builder)
        arm.defaultCommand = idle
        assertEquals(ConflictBehavior.CANCEL, builder.conflictBehavior())
        tick()
        assertTrue(Scheduler.isScheduled(idle))
        assertFalse(Scheduler.isScheduled(builder))

        val lift = logged("Lift", armCommand())
        Scheduler.schedule(lift)
        tick()
        tick()
        Scheduler.cancel(builder)
        assertTrue(Scheduler.isScheduled(lift))
        Scheduler.cancel(lift)
        tick()

        assertEquals(
            listOf("START #1 Arm idle", "INTERRUPT #1 Arm idle", "START #2 Lift", "INTERRUPT #2 Lift", "START #3 Arm idle", "ABORT #3 Arm idle: op-mode stop"),
            log().commandEvents(),
        )
    }

    @Test
    fun suspensionAndResumption() {
        val low = logged("Patrol", armCommand().setInterruptedBehavior(InterruptedBehavior.SUSPEND))
        Scheduler.schedule(low)
        tick()
        var urgentDone = false
        Scheduler.schedule(logged("Urgent", armCommand { urgentDone }.setPriority(CommandPriorities.DRIVER_ACTION)))
        tick()
        urgentDone = true
        repeat(3) { tick() }
        Scheduler.cancel(low)

        val log = log()
        assertEquals(
            listOf("START #1 Patrol", "SUSPEND #1 Patrol", "START #2 Urgent", "FINISH #2 Urgent", "RESUME #1 Patrol", "INTERRUPT #1 Patrol"),
            log.commandEvents(),
        )
        assertTrue("#1 Patrol (suspended)\n#2 Urgent" in log.active())
    }

    @Test
    fun anInstantBetweenRecorderSamplesStillHasItsOwnStartAndFinish() {
        tick(ms = 2.0)
        tick(ms = 2.0, input = { Scheduler.schedule(logged("Blip", instant {})) })
        tick(ms = 2.0)

        val log = log()
        val events = log.strings("commands/events")
        assertEquals(listOf("START #1 Blip", "FINISH #1 Blip"), events.map { it.second })
        assertEquals(4_000L, events[0].first)
        assertTrue(events[1].first > events[0].first)
        // The ticks were closer together than the sampling interval.
        assertTrue(log.longs("loop/totalNanos").size < 3)
        assertEquals(listOf("", "#1 Blip", ""), log.active())
    }

    @Test
    fun opModeStopClosesOpenExecutionsAfterHardwareWithoutRunningEndHandlers() {
        var endCalls = 0
        Scheduler.schedule(logged("Lift", armCommand().setEnd { endCalls++ }))
        tick()
        val stopUs = (clock.nanos() - FakeClock().nanos()) / 1000

        val log = log()
        assertEquals(1, arm.stopCalls)
        assertEquals(0, endCalls)
        val abort = log.strings("commands/events").last()
        assertEquals("ABORT #1 Lift: op-mode stop", abort.second)
        assertTrue(abort.first >= stopUs + 5_000)
        assertEquals("", log.active().last())
    }

    @Test
    fun anErrorEscapesAndTheCrashPathClosesTheRestAsStop() {
        Scheduler.schedule(logged("Bystander", infinite {}))
        Scheduler.schedule(logged("Unfinished", armCommand().setExecute { TODO() }))
        val crash = assertThrows(NotImplementedError::class.java) { tick() }
        robot.stopAfterCrash(crash)

        assertEquals(0, robot.commandFaultCount)
        assertEquals(
            listOf(
                "START #1 Bystander",
                "START #2 Unfinished",
                "FAIL #2 Unfinished in execute: NotImplementedError: An operation is not implemented.",
                "ABORT #1 Bystander: op-mode stop",
            ),
            log().commandEvents(),
        )
    }

    @Test
    fun boundedStateReportsLostHistoryAndNeverChangesWhatCommandsDo() {
        var runs = 0
        val many = List(CommandHistory.MAX_OPEN + 6) { logged("Worker", infinite { runs++ }) }
        many.forEach(Scheduler::schedule)
        tick()
        assertEquals(many.size, runs)
        assertTrue(many.all(Scheduler::isScheduled))

        many.forEach(Scheduler::cancel)
        // More lifecycle records between two drains than the queue holds.
        val flicker = logged("Flicker", infinite {})
        repeat(CommandHistory.MAX_PENDING) {
            Scheduler.schedule(flicker)
            Scheduler.cancel(flicker)
        }
        assertFalse(Scheduler.isScheduled(flicker))
        tick()

        val log = log()
        val lost = log.longs("commands/lost").last().second
        assertTrue(lost >= 6 + CommandHistory.MAX_OPEN)
        assertTrue(log.commandEvents().any { it.startsWith("HISTORY INCOMPLETE: ") })
        assertEquals(CommandHistory.MAX_OPEN, log.commandEvents().count { it.startsWith("START #") && it.endsWith("Worker") })
    }

    @Test
    fun aRecorderIoFailureChangesNothingForCommandsOrStopping() {
        var failing = false
        val fresh = Robot(HardwareMap(null, null), clock)
        val flaky = fresh.register(object : SubsystemBase("Flaky") {
            var stopped = false
            override fun logState(log: StateLog) {
                if (failing) throw UncheckedIOException(IOException("sd card removed"))
            }
            override fun stop() { stopped = true }
        })
        fresh.enableFlightRecorder("Flaky", { null }, { null }, { null }, directory = logDir)
        fresh.init()
        fresh.start()
        var executions = 0
        var finish = false
        val step = logged("Step", Command.build().requiring(flaky).setExecute { executions++ }.setDone { finish })
        Scheduler.schedule(step)
        clock.advanceMs(20.0)
        fresh.loop()
        failing = true
        repeat(3) {
            clock.advanceMs(20.0)
            fresh.loop()
        }
        finish = true
        clock.advanceMs(20.0)
        fresh.loop()

        assertEquals(5, executions)
        assertFalse(Scheduler.isScheduled(step))
        assertEquals(0, fresh.commandFaultCount)
        fresh.stop()
        assertTrue(flaky.stopped)
    }
}
