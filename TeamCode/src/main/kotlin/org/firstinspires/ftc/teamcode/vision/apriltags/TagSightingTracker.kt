package org.firstinspires.ftc.teamcode.vision.apriltags

import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightFiducial

/**
 * Per-tag sighting history for the AprilTag diagnostic: when each ID was last
 * in a fresh result and how many fresh frames contained it.
 *
 * Feed it only the fiducials of a fresh, matching-pipeline result, once per new
 * device frame ([LimelightSubsystem][org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem]
 * already clears them otherwise). Times are robot-loop nanoTime at the tick the
 * frame was first seen — receipt time, not acquisition time.
 */
class TagSightingTracker {

    data class Sighting(
        val id: Int,
        val catalogTag: BiobuzzAprilTags.Tag?,
        val lastSeenNs: Long,
        val framesSeen: Long,
        val latest: LimelightFiducial,
    ) {
        fun ageMs(nowNs: Long): Double = (nowNs - lastSeenNs) / 1e6
    }

    private val sightings = LinkedHashMap<Int, Sighting>()

    var framesObserved: Long = 0
        private set

    fun recordFrame(fiducials: List<LimelightFiducial>, nowNs: Long) {
        framesObserved++
        for (fiducial in fiducials) {
            val previous = sightings[fiducial.id]
            sightings[fiducial.id] = Sighting(
                id = fiducial.id,
                catalogTag = BiobuzzAprilTags.lookup(fiducial.id),
                lastSeenNs = nowNs,
                framesSeen = (previous?.framesSeen ?: 0L) + 1,
                latest = fiducial,
            )
        }
    }

    fun sighting(id: Int): Sighting? = sightings[id]

    /** Every ID ever seen this run, season catalog first then unknown IDs, ascending. */
    fun all(): List<Sighting> = sightings.values.sortedWith(
        compareBy<Sighting> { it.catalogTag == null }.thenBy { it.id },
    )

    /** Newest sighting of any tag on [cell], or null if none has been seen this run. */
    fun latestFor(cell: BiobuzzAprilTags.Cell): Sighting? =
        sightings.values.filter { it.catalogTag?.cell == cell }.maxByOrNull { it.lastSeenNs }

    fun reset() {
        sightings.clear()
        framesObserved = 0
    }
}
