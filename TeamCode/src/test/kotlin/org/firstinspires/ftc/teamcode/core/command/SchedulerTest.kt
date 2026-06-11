package org.firstinspires.ftc.teamcode.core.command

import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Test

/**
 * Conformance suite for the owned scheduler. The priority semantics here are
 * load-bearing for the whole framework (CommandPriorities ladder): blocked
 * only by strictly-higher priority, equal preempts, end handlers always run.
 */
class SchedulerTest {

    private val scheduler = Scheduler()

    private class Probe(
        requirement: Any? = null,
        priority: Int = 0,
        done: () -> Boolean = { false },
    ) : Command {
        private val requirements = if (requirement != null) setOf(requirement) else emptySet()
        private val priorityValue = priority
        private val doneCondition = done
        var starts = 0
        var executes = 0
        var ends = mutableListOf<EndCondition>()

        override fun requirements(): Set<Any> = requirements
        override fun priority(): Int = priorityValue
        override fun start() { starts++ }
        override fun execute() { executes++ }
        override fun done(): Boolean = doneCondition()
        override fun end(endCondition: EndCondition) { ends += endCondition }
    }

    // ------------------------------------------------------------- scheduling

    @Test
    fun scheduleStartsImmediatelyAndExecutesPerTick() {
        val cmd = Probe()
        assertTrue(scheduler.schedule(cmd))
        assertEquals(1, cmd.starts)
        assertEquals(0, cmd.executes)

        scheduler.execute()
        scheduler.execute()
        assertEquals(2, cmd.executes)
    }

    @Test
    fun schedulingAnAlreadyScheduledCommandIsANoOp() {
        val cmd = Probe()
        scheduler.schedule(cmd)
        assertTrue(scheduler.schedule(cmd))
        assertEquals(1, cmd.starts)
    }

    @Test
    fun naturalCompletionEndsWithNaturallyAndReleasesRequirements() {
        val req = Any()
        var finished = false
        val cmd = Probe(req, done = { finished })
        scheduler.schedule(cmd)

        finished = true
        scheduler.execute()

        assertEquals(listOf(EndCondition.NATURALLY), cmd.ends)
        assertFalse(scheduler.isScheduled(cmd))
        // Requirement is free again.
        assertTrue(scheduler.schedule(Probe(req)))
    }

    // --------------------------------------------------------------- priority

    @Test
    fun strictlyHigherPriorityHolderBlocksNewCommand() {
        val req = Any()
        val holder = Probe(req, priority = 20)
        val intruder = Probe(req, priority = 10)
        scheduler.schedule(holder)

        assertFalse(scheduler.schedule(intruder))
        assertTrue(scheduler.isScheduled(holder))
        assertFalse(scheduler.isScheduled(intruder))
        assertEquals(0, intruder.starts)
    }

    @Test
    fun equalPriorityPreemptsTheHolder() {
        val req = Any()
        val holder = Probe(req, priority = 20)
        val intruder = Probe(req, priority = 20)
        scheduler.schedule(holder)

        assertTrue(scheduler.schedule(intruder))
        assertEquals(listOf(EndCondition.INTERRUPTED), holder.ends)
        assertFalse(scheduler.isScheduled(holder))
        assertTrue(scheduler.isScheduled(intruder))
    }

    @Test
    fun higherPriorityInterruptsAllLowerHolders() {
        val reqA = Any()
        val reqB = Any()
        val holderA = Probe(reqA, priority = 0)
        val holderB = Probe(reqB, priority = 10)
        scheduler.schedule(holderA)
        scheduler.schedule(holderB)

        val both = object : Command {
            override fun requirements(): Set<Any> = setOf(reqA, reqB)
            override fun priority(): Int = 20
            override fun start() {}
            override fun execute() {}
            override fun done(): Boolean = false
            override fun end(endCondition: EndCondition) {}
        }
        assertTrue(scheduler.schedule(both))
        assertEquals(listOf(EndCondition.INTERRUPTED), holderA.ends)
        assertEquals(listOf(EndCondition.INTERRUPTED), holderB.ends)
    }

    // ------------------------------------------------------- cancel and reset

    @Test
    fun cancelRunsEndHandlerWithInterrupted() {
        val cmd = Probe()
        scheduler.schedule(cmd)
        scheduler.cancel(cmd)
        assertEquals(listOf(EndCondition.INTERRUPTED), cmd.ends)
        assertFalse(scheduler.isScheduled(cmd))
    }

    @Test
    fun cancelOfUnscheduledCommandIsSafe() {
        val cmd = Probe()
        scheduler.cancel(cmd) // must not throw
        assertTrue(cmd.ends.isEmpty())
    }

    @Test
    fun resetInterruptsEverythingAndRunsEndHandlers() {
        val a = Probe(Any())
        val b = Probe(Any())
        scheduler.schedule(a)
        scheduler.schedule(b)

        scheduler.reset()

        assertEquals(listOf(EndCondition.INTERRUPTED), a.ends)
        assertEquals(listOf(EndCondition.INTERRUPTED), b.ends)
        assertTrue(scheduler.runningCommands().isEmpty())
    }

    @Test
    fun resetSwallowsThrowingEndHandlers() {
        val bad = Command.build().setEnd { error("end blew up") }
        val good = Probe()
        scheduler.schedule(bad)
        scheduler.schedule(good)

        scheduler.reset() // must not throw

        assertEquals(listOf(EndCondition.INTERRUPTED), good.ends)
    }

    // ----------------------------------------------------------------- faults

    @Test
    fun faultWithoutHandlerPropagates() {
        val bad = Command.build().setExecute { error("boom") }
        scheduler.schedule(bad)
        try {
            scheduler.execute()
            fail("expected the fault to propagate")
        } catch (e: IllegalStateException) {
            assertEquals("boom", e.message)
        }
        assertFalse(scheduler.isScheduled(bad))
    }

    @Test
    fun faultIsContainedPerCommand() {
        var faulted: Command? = null
        scheduler.faultHandler = { cmd, _ -> faulted = cmd }
        val bad = Command.build().setExecute { error("boom") }
        val good = Probe()
        scheduler.schedule(bad)
        scheduler.schedule(good)

        scheduler.execute()

        assertEquals(bad, faulted)
        assertFalse(scheduler.isScheduled(bad))
        assertTrue(scheduler.isScheduled(good))
        assertEquals(1, good.executes)
    }

    @Test
    fun faultedCommandGetsFaultedEndConditionAndFreesRequirements() {
        scheduler.faultHandler = { _, _ -> }
        val req = Any()
        var ended: EndCondition? = null
        val bad = Command.build()
            .requiring(req)
            .setExecute { error("boom") }
            .setEnd { ended = it }
        scheduler.schedule(bad)
        scheduler.execute()

        assertEquals(EndCondition.FAULTED, ended)
        assertTrue(scheduler.schedule(Probe(req)))
    }

    @Test
    fun faultInStartReportsAndDoesNotSchedule() {
        var faulted: Command? = null
        scheduler.faultHandler = { cmd, _ -> faulted = cmd }
        val bad = Command.build().setStart { error("start boom") }

        assertFalse(scheduler.schedule(bad))
        assertEquals(bad, faulted)
        assertFalse(scheduler.isScheduled(bad))
    }

    @Test
    fun throwingEndHandlerOnNaturalCompletionIsReportedAsFault() {
        var faulted: Command? = null
        scheduler.faultHandler = { cmd, _ -> faulted = cmd }
        val bad = Command.build().setDone { true }.setEnd { error("end boom") }
        scheduler.schedule(bad)

        scheduler.execute()

        assertEquals(bad, faulted)
        assertFalse(scheduler.isScheduled(bad))
    }

    // ------------------------------------------------------------------ reuse

    @Test
    fun commandsAreReusableAcrossRuns() {
        var finished = false
        val cmd = Probe(done = { finished })
        scheduler.schedule(cmd)
        finished = true
        scheduler.execute()

        finished = false
        scheduler.schedule(cmd)
        scheduler.execute()

        assertEquals(2, cmd.starts)
        assertEquals(listOf(EndCondition.NATURALLY), cmd.ends)
        assertTrue(scheduler.isScheduled(cmd))
    }

    // ----------------------------------------------------------------- groups

    @Test
    fun sequentialRunsChildrenInOrderWithUnionRequirements() {
        val reqA = Any()
        val reqB = Any()
        var aDone = false
        val a = Probe(reqA, done = { aDone })
        val b = Probe(reqB)
        val group = Groups.sequential(a, b)

        assertEquals(setOf(reqA, reqB), group.requirements())
        scheduler.schedule(group)
        assertEquals(1, a.starts)
        assertEquals(0, b.starts)

        scheduler.execute()
        assertEquals(0, b.starts)

        aDone = true
        scheduler.execute()
        assertEquals(listOf(EndCondition.NATURALLY), a.ends)
        assertEquals(1, b.starts)
        assertTrue(scheduler.isScheduled(group))
    }

    @Test
    fun interruptedSequentialForwardsToCurrentChild() {
        var aDone = false
        val a = Probe(done = { aDone })
        val b = Probe()
        val group = Groups.sequential(a, b)
        scheduler.schedule(group)

        aDone = true
        scheduler.execute() // a completes, b starts
        scheduler.cancel(group)

        assertEquals(listOf(EndCondition.NATURALLY), a.ends)
        assertEquals(listOf(EndCondition.INTERRUPTED), b.ends)
    }

    @Test
    fun parallelCompletesWhenAllChildrenHave() {
        var aDone = false
        var bDone = false
        val a = Probe(done = { aDone })
        val b = Probe(done = { bDone })
        val group = Groups.parallel(a, b)
        scheduler.schedule(group)

        aDone = true
        scheduler.execute()
        assertTrue(scheduler.isScheduled(group))
        assertEquals(listOf(EndCondition.NATURALLY), a.ends)

        bDone = true
        scheduler.execute()
        assertFalse(scheduler.isScheduled(group))
        assertEquals(listOf(EndCondition.NATURALLY), b.ends)
        // a finished earlier; it must not be ended twice.
        assertEquals(1, a.ends.size)
    }

    @Test
    fun raceEndsWinnerNaturallyAndInterruptsTheRest() {
        var aDone = false
        val a = Probe(done = { aDone })
        val b = Probe()
        val group = Groups.race(a, b)
        scheduler.schedule(group)

        aDone = true
        scheduler.execute()

        assertFalse(scheduler.isScheduled(group))
        assertEquals(listOf(EndCondition.NATURALLY), a.ends)
        assertEquals(listOf(EndCondition.INTERRUPTED), b.ends)
    }

    @Test
    fun deadlineInterruptsOthersWhenItCompletes() {
        var deadlineDone = false
        val deadline = Probe(done = { deadlineDone })
        val worker = Probe()
        val group = Groups.deadline(deadline, worker)
        scheduler.schedule(group)

        scheduler.execute()
        assertTrue(scheduler.isScheduled(group))

        deadlineDone = true
        scheduler.execute()
        assertFalse(scheduler.isScheduled(group))
        assertEquals(listOf(EndCondition.NATURALLY), deadline.ends)
        assertEquals(listOf(EndCondition.INTERRUPTED), worker.ends)
    }

    @Test
    fun deadlineDoesNotInterruptAlreadyFinishedWorkers() {
        var deadlineDone = false
        var workerDone = false
        val deadline = Probe(done = { deadlineDone })
        val worker = Probe(done = { workerDone })
        val group = Groups.deadline(deadline, worker)
        scheduler.schedule(group)

        workerDone = true
        scheduler.execute()
        assertEquals(listOf(EndCondition.NATURALLY), worker.ends)

        deadlineDone = true
        scheduler.execute()
        assertEquals(1, worker.ends.size)
    }

    // --------------------------------------------------------------- commands

    @Test
    fun waitMsRunsOnTheInjectedClock() {
        val clock = FakeClock()
        val wait = Commands.waitMs(100.0, clock)
        scheduler.schedule(wait)

        scheduler.execute()
        assertTrue(scheduler.isScheduled(wait))

        clock.advanceMs(150.0)
        scheduler.execute()
        assertFalse(scheduler.isScheduled(wait))
    }

    @Test
    fun instantRunsItsActionOnceInStart() {
        var runs = 0
        val cmd = Commands.instant { runs++ }
        scheduler.schedule(cmd)
        assertEquals(1, runs)
        scheduler.execute()
        assertEquals(1, runs)
        assertFalse(scheduler.isScheduled(cmd))
    }

    @Test
    fun deferBuildsTheCommandAtScheduleTimeAndForwardsLifecycle() {
        val req = Any()
        var built = 0
        var innerEnds: EndCondition? = null
        val deferred = Commands.defer(req) {
            built++
            Command.build()
                .setDone { false }
                .setEnd { innerEnds = it }
        }

        assertEquals(0, built)
        scheduler.schedule(deferred)
        assertEquals(1, built)
        assertEquals(setOf(req), deferred.requirements())

        scheduler.cancel(deferred)
        assertEquals(EndCondition.INTERRUPTED, innerEnds)

        // Rescheduling builds a fresh inner command.
        scheduler.schedule(deferred)
        assertEquals(2, built)
    }

    @Test
    fun introspectionReportsRunningCommandNames() {
        assertNull(scheduler.runningCommands().firstOrNull())
        val named = Command.build().setName("lift→24.0").setDone { false }
        scheduler.schedule(named)
        assertEquals(listOf("lift→24.0"), scheduler.runningCommandNames())
    }
}
