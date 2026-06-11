package org.firstinspires.ftc.teamcode.core.runtime

import dev.frozenmilk.sinister.loading.Pinned
import java.io.File

/**
 * Small pinned pose handoff between op-modes.
 *
 * Only primitives live here so Sloth can keep this object resident across
 * hot reloads without retaining a stale geometry class instance.
 *
 * Every [record] is also mirrored to disk (best-effort), and
 * [restoreFromDiskIfNeeded] reads it back when the in-memory copy is empty —
 * so the auton→teleop handoff survives a Robot Controller process restart
 * (DS "Restart Robot", a crash, a brownout). [wallTimeMs] still bounds how
 * old a restored pose may be.
 */
@Pinned
object PersistedPose {
    private const val MAGIC = "pose-v1"

    /** Overridable for host tests; null disables disk persistence entirely. */
    internal var storageFile: File? = File("/sdcard/FIRST/persisted-pose.txt")

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
    fun record(pose: org.firstinspires.ftc.teamcode.core.geometry.Pose2d) {
        valid = true
        x = pose.x
        y = pose.y
        headingRad = pose.heading
        wallTimeMs = System.currentTimeMillis()
        writeToDisk()
    }

    /** Clear the saved pose (memory and disk) so future restores fail closed. */
    fun clear() {
        valid = false
        x = 0.0
        y = 0.0
        headingRad = 0.0
        wallTimeMs = 0L
        try {
            storageFile?.delete()
        } catch (_: Throwable) {
            // Best-effort: a stale file is still age-gated on restore.
        }
    }

    /**
     * Load the on-disk pose if the in-memory copy is empty (i.e. the process
     * restarted since [record]). No-op when memory already holds a pose.
     */
    fun restoreFromDiskIfNeeded() {
        if (valid) return
        val file = storageFile ?: return
        try {
            if (!file.exists()) return
            val parts = file.readText().trim().split(" ")
            if (parts.size != 5 || parts[0] != MAGIC) return
            val fileX = parts[1].toDouble()
            val fileY = parts[2].toDouble()
            val fileHeading = parts[3].toDouble()
            val fileWallTimeMs = parts[4].toLong()
            if (!fileX.isFinite() || !fileY.isFinite() || !fileHeading.isFinite()) return
            x = fileX
            y = fileY
            headingRad = fileHeading
            wallTimeMs = fileWallTimeMs
            valid = true
        } catch (_: Throwable) {
            // Unreadable file == no persisted pose.
        }
    }

    private fun writeToDisk() {
        val file = storageFile ?: return
        try {
            file.parentFile?.mkdirs()
            val tmp = File(file.path + ".tmp")
            tmp.writeText("$MAGIC $x $y $headingRad $wallTimeMs")
            if (!tmp.renameTo(file)) {
                file.delete()
                tmp.renameTo(file)
            }
        } catch (_: Throwable) {
            // Best-effort: the in-memory handoff still works without disk.
        }
    }

    /** Simulate a process restart in host tests: wipe memory, keep disk. */
    internal fun forgetInMemoryForTest() {
        valid = false
        x = 0.0
        y = 0.0
        headingRad = 0.0
        wallTimeMs = 0L
    }
}
