package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.geometry.Pose
import java.io.File
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class PersistedPoseTest {

    private lateinit var tempFile: File
    private var originalFile: File? = null

    @Before
    fun setUp() {
        tempFile = File.createTempFile("persisted-pose", ".txt").also { it.delete() }
        originalFile = PersistedPose.storageFile
        PersistedPose.storageFile = tempFile
        PersistedPose.clear()
    }

    @After
    fun tearDown() {
        PersistedPose.clear()
        PersistedPose.storageFile = originalFile
        tempFile.delete()
    }

    @Test
    fun recordSurvivesAProcessRestart() {
        PersistedPose.record(Pose(12.5, -3.25, 1.5))
        PersistedPose.forgetInMemoryForTest()
        assertFalse(PersistedPose.valid)

        PersistedPose.restoreFromDiskIfNeeded()

        assertTrue(PersistedPose.valid)
        assertEquals(12.5, PersistedPose.x, 1e-12)
        assertEquals(-3.25, PersistedPose.y, 1e-12)
        assertEquals(1.5, PersistedPose.headingRad, 1e-12)
        assertTrue(PersistedPose.wallTimeMs > 0L)
    }

    @Test
    fun restoreDoesNotClobberALivePose() {
        PersistedPose.record(Pose(1.0, 2.0, 3.0))
        tempFile.writeText("pose-v1 9.0 9.0 9.0 1")

        PersistedPose.restoreFromDiskIfNeeded()

        assertEquals(1.0, PersistedPose.x, 1e-12)
    }

    @Test
    fun clearRemovesTheDiskCopy() {
        PersistedPose.record(Pose(1.0, 2.0, 3.0))
        assertTrue(tempFile.exists())

        PersistedPose.clear()
        PersistedPose.restoreFromDiskIfNeeded()

        assertFalse(PersistedPose.valid)
        assertFalse(tempFile.exists())
    }

    @Test
    fun corruptFileFailsClosed() {
        tempFile.writeText("not a pose at all")
        PersistedPose.restoreFromDiskIfNeeded()
        assertFalse(PersistedPose.valid)

        tempFile.writeText("pose-v1 NaN 2.0 3.0 1")
        PersistedPose.restoreFromDiskIfNeeded()
        assertFalse(PersistedPose.valid)

        tempFile.writeText("pose-v2 1.0 2.0 3.0 1")
        PersistedPose.restoreFromDiskIfNeeded()
        assertFalse(PersistedPose.valid)
    }

    @Test
    fun missingFileIsANoOp() {
        PersistedPose.restoreFromDiskIfNeeded()
        assertFalse(PersistedPose.valid)
    }
}
