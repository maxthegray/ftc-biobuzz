package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.geometry.Pose
import dev.frozenmilk.sinister.loading.Pinned

/**
 * Small pinned pose handoff between op-modes.
 *
 * Only primitives live here so Sloth can keep this object resident across
 * hot reloads without retaining a stale Pedro class instance.
 */
@Pinned
object PersistedPose {
    var valid: Boolean = false
        private set
    var x: Double = 0.0
        private set
    var y: Double = 0.0
        private set
    var headingRad: Double = 0.0
        private set
    var wallTimeMs: Long = 0L
        private set

    /** Record the latest field pose for a later op-mode to restore. */
    fun record(pose: Pose) {
        valid = true
        x = pose.x
        y = pose.y
        headingRad = pose.heading
        wallTimeMs = System.currentTimeMillis()
    }

    /** Clear the saved pose so future restores fail closed. */
    fun clear() {
        valid = false
        x = 0.0
        y = 0.0
        headingRad = 0.0
        wallTimeMs = 0L
    }
}
