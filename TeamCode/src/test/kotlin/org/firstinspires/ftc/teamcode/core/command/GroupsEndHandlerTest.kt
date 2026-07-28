package org.firstinspires.ftc.teamcode.core.command

import org.junit.Assert.assertEquals
import org.junit.Assert.assertThrows
import org.junit.Test

/**
 * One throwing child end handler must not skip its siblings' cleanup — the
 * scheduler's "every end handler runs" guarantee has to hold inside groups
 * too (a skipped drive-command end handler would leave a follow un-broken).
 */
class GroupsEndHandlerTest {

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
}
