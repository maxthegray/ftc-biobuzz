package org.firstinspires.ftc.teamcode.core.command

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * One throwing child end handler must not skip its siblings' cleanup — the
 * scheduler's "every end handler runs" guarantee has to hold inside groups
 * too (a skipped drive-command end handler would leave a follow un-broken).
 */
class GroupsEndHandlerTest {

    private enum class LifecyclePhase {
        START,
        EXECUTE,
        DONE,
        END,
    }

    private fun faulting(
        ends: MutableMap<String, MutableList<EndCondition>>,
        name: String,
        phase: LifecyclePhase,
    ) = CommandBuilder()
        .setName(name)
        .setStart { if (phase == LifecyclePhase.START) error("$name start failed") }
        .setExecute { if (phase == LifecyclePhase.EXECUTE) error("$name execute failed") }
        .setDone {
            if (phase == LifecyclePhase.DONE) error("$name done failed")
            phase == LifecyclePhase.END
        }
        .setEnd {
            ends.getOrPut(name, ::mutableListOf) += it
            if (phase == LifecyclePhase.END) error("$name end failed")
        }

    private fun running(
        ends: MutableMap<String, MutableList<EndCondition>>,
        name: String,
    ) = CommandBuilder()
        .setName(name)
        .setDone { false }
        .setEnd { ends.getOrPut(name, ::mutableListOf) += it }

    private fun assertEnds(
        ends: Map<String, List<EndCondition>>,
        name: String,
        vararg expected: EndCondition,
    ) {
        assertEquals(expected.toList(), ends[name].orEmpty())
    }

    private fun scheduler() = Scheduler().apply {
        faultHandler = { _, _ -> }
    }

    private fun recording(ends: MutableList<String>, name: String) =
        CommandBuilder()
            .setName(name)
            .setDone { false }
            .setEnd { ends += "$name:$it" }

    private fun throwingEnd(name: String) =
        CommandBuilder()
            .setName(name)
            .setDone { false }
            .setEnd { error("end of $name failed") }

    @Test
    fun parallelEndsEveryChildEvenWhenOneEndThrows() {
        val ends = mutableListOf<String>()
        val group = Groups.parallel(
            throwingEnd("a"),
            recording(ends, "b"),
            recording(ends, "c"),
        )
        group.start()

        assertThrows(IllegalStateException::class.java) {
            group.end(EndCondition.INTERRUPTED)
        }
        assertEquals(listOf("b:INTERRUPTED", "c:INTERRUPTED"), ends)
    }

    @Test
    fun deadlineEndsOthersEvenWhenTheDeadlineEndThrows() {
        val ends = mutableListOf<String>()
        val group = Groups.deadline(
            throwingEnd("deadline"),
            recording(ends, "worker"),
        )
        group.start()

        assertThrows(IllegalStateException::class.java) {
            group.end(EndCondition.INTERRUPTED)
        }
        assertEquals(listOf("worker:INTERRUPTED"), ends)
    }

    @Test
    fun raceEndsLosersEvenWhenTheWinnerEndThrows() {
        val ends = mutableListOf<String>()
        val winner = CommandBuilder()
            .setName("winner")
            .setDone { true }
            .setEnd { error("winner end failed") }
        val group = Groups.race(winner, recording(ends, "loser"))
        group.start()
        group.execute()

        assertThrows(IllegalStateException::class.java) {
            group.end(EndCondition.NATURALLY)
        }
        assertEquals(listOf("loser:INTERRUPTED"), ends)
    }

    @Test
    fun parallelDoesNotEndAChildTwiceWhenItsNaturalEndThrows() {
        var aEnds = 0
        val a = CommandBuilder()
            .setName("a")
            .setDone { true }
            .setEnd {
                aEnds++
                error("boom")
            }
        val ends = mutableListOf<String>()
        val group = Groups.parallel(a, recording(ends, "b"))
        group.start()

        // The throw propagates: the scheduler would fault the whole group…
        assertThrows(IllegalStateException::class.java) { group.execute() }
        // …and the group's end handler must not end the finished child again.
        group.end(EndCondition.FAULTED)

        assertEquals(1, aEnds)
        assertEquals(listOf("b:FAULTED"), ends)
    }

    @Test
    fun sequentialFaultsEndOnlyTheStartedChildOnce() {
        for (phase in LifecyclePhase.entries) {
            val ends = mutableMapOf<String, MutableList<EndCondition>>()
            val group = Groups.sequential(
                faulting(ends, "fault", phase),
                running(ends, "later"),
            )
            val scheduler = scheduler()

            val scheduled = scheduler.schedule(group)
            assertEquals(phase != LifecyclePhase.START, scheduled)
            if (scheduled) scheduler.execute()

            assertFalse(scheduler.isScheduled(group))
            assertEnds(
                ends,
                "fault",
                if (phase == LifecyclePhase.END) {
                    EndCondition.NATURALLY
                } else {
                    EndCondition.FAULTED
                },
            )
            assertEnds(ends, "later")
        }
    }

    @Test
    fun parallelFaultsEndStartedChildrenOnceAndSkipNeverStartedChildren() {
        for (phase in LifecyclePhase.entries) {
            val ends = mutableMapOf<String, MutableList<EndCondition>>()
            val group = Groups.parallel(
                running(ends, "before"),
                faulting(ends, "fault", phase),
                running(ends, "after"),
            )
            val scheduler = scheduler()

            val scheduled = scheduler.schedule(group)
            assertEquals(phase != LifecyclePhase.START, scheduled)
            if (scheduled) scheduler.execute()

            assertFalse(scheduler.isScheduled(group))
            assertEnds(ends, "before", EndCondition.FAULTED)
            assertEnds(
                ends,
                "fault",
                if (phase == LifecyclePhase.END) {
                    EndCondition.NATURALLY
                } else {
                    EndCondition.FAULTED
                },
            )
            if (phase == LifecyclePhase.START) {
                assertEnds(ends, "after")
            } else {
                assertEnds(ends, "after", EndCondition.FAULTED)
            }
        }
    }

    @Test
    fun raceFaultsEndStartedChildrenOnceAndSkipNeverStartedChildren() {
        for (phase in LifecyclePhase.entries) {
            val ends = mutableMapOf<String, MutableList<EndCondition>>()
            val group = Groups.race(
                running(ends, "before"),
                faulting(ends, "fault", phase),
                running(ends, "after"),
            )
            val scheduler = scheduler()

            val scheduled = scheduler.schedule(group)
            assertEquals(phase != LifecyclePhase.START, scheduled)
            if (scheduled) scheduler.execute()

            assertFalse(scheduler.isScheduled(group))
            assertEnds(
                ends,
                "before",
                if (phase == LifecyclePhase.END) {
                    EndCondition.INTERRUPTED
                } else {
                    EndCondition.FAULTED
                },
            )
            assertEnds(
                ends,
                "fault",
                if (phase == LifecyclePhase.END) {
                    EndCondition.NATURALLY
                } else {
                    EndCondition.FAULTED
                },
            )
            if (phase == LifecyclePhase.START) {
                assertEnds(ends, "after")
            } else {
                assertEnds(
                    ends,
                    "after",
                    if (phase == LifecyclePhase.END) {
                        EndCondition.INTERRUPTED
                    } else {
                        EndCondition.FAULTED
                    },
                )
            }
        }
    }

    @Test
    fun deadlineFaultsEndStartedChildrenOnceAndSkipNeverStartedChildren() {
        for (phase in LifecyclePhase.entries) {
            val ends = mutableMapOf<String, MutableList<EndCondition>>()
            val group = Groups.deadline(
                running(ends, "deadline"),
                running(ends, "before"),
                faulting(ends, "fault", phase),
                running(ends, "after"),
            )
            val scheduler = scheduler()

            val scheduled = scheduler.schedule(group)
            assertEquals(phase != LifecyclePhase.START, scheduled)
            if (scheduled) scheduler.execute()

            assertFalse(scheduler.isScheduled(group))
            assertEnds(ends, "deadline", EndCondition.FAULTED)
            assertEnds(ends, "before", EndCondition.FAULTED)
            assertEnds(
                ends,
                "fault",
                if (phase == LifecyclePhase.END) {
                    EndCondition.NATURALLY
                } else {
                    EndCondition.FAULTED
                },
            )
            if (phase == LifecyclePhase.START) {
                assertEnds(ends, "after")
            } else {
                assertEnds(ends, "after", EndCondition.FAULTED)
            }
        }
    }
}
