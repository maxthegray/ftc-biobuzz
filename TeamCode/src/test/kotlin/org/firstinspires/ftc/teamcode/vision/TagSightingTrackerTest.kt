package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightFiducial
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class TagSightingTrackerTest {

    @Test
    fun recordsLastSeenAndFrameCountsPerTag() {
        val tracker = TagSightingTracker()
        tracker.recordFrame(listOf(fiducial(34), fiducial(35)), nowNs = 1_000_000_000)
        tracker.recordFrame(emptyList(), nowNs = 1_050_000_000)
        tracker.recordFrame(listOf(fiducial(34, tx = 2.0)), nowNs = 1_100_000_000)

        assertEquals(3, tracker.framesObserved)
        val s34 = tracker.sighting(34)!!
        assertEquals(2, s34.framesSeen)
        assertEquals(2.0, s34.latest.txDegrees, 0.0)
        assertEquals(50.0, s34.ageMs(1_150_000_000), 1e-9)
        assertEquals(1, tracker.sighting(35)!!.framesSeen)
        assertEquals("RED AUDIENCE slot 1", s34.catalogTag!!.meaning)
    }

    @Test
    fun unknownTagsSortAfterSeasonTagsAndHaveNoCell() {
        val tracker = TagSightingTracker()
        tracker.recordFrame(listOf(fiducial(7), fiducial(44), fiducial(31)), nowNs = 1)

        assertEquals(listOf(31, 44, 7), tracker.all().map { it.id })
        assertNull(tracker.sighting(7)!!.catalogTag)
    }

    @Test
    fun latestForCellUsesOnlyThatCellsTags() {
        val tracker = TagSightingTracker()
        tracker.recordFrame(listOf(fiducial(30)), nowNs = 10)
        tracker.recordFrame(listOf(fiducial(33)), nowNs = 20)
        tracker.recordFrame(listOf(fiducial(34)), nowNs = 30)

        val redFar = BiobuzzAprilTags.lookup(30)!!.cell
        assertEquals(33, tracker.latestFor(redFar)!!.id)
        assertNull(tracker.latestFor(BiobuzzAprilTags.lookup(42)!!.cell))
    }

    @Test
    fun resetForgetsEverything() {
        val tracker = TagSightingTracker()
        tracker.recordFrame(listOf(fiducial(30)), nowNs = 10)
        tracker.reset()
        assertTrue(tracker.all().isEmpty())
        assertEquals(0, tracker.framesObserved)
    }

    private fun fiducial(id: Int, tx: Double = 0.0) = LimelightFiducial(
        id = id, family = "36H11C", txDegrees = tx, tyDegrees = 0.0,
        txNoCrosshairDegrees = tx, tyNoCrosshairDegrees = 0.0, areaPercent = 1.0,
        targetPoseCameraSpace = null, cameraPoseTargetSpace = null, cornerCount = 4,
    )
}
