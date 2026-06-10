package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.geometry.Pose
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test
import kotlin.math.PI

class PoseHistoryTest {

    @Test
    fun lookupInterpolatesBetweenSamples() {
        val history = PoseHistory(capacity = 4)
        history.add(0L, Pose(0.0, 0.0, 0.0))
        history.add(100L, Pose(10.0, 20.0, PI))

        val pose = history.lookup(50L)!!

        assertEquals(5.0, pose.x, EPS)
        assertEquals(10.0, pose.y, EPS)
        assertEquals(PI / 2.0, pose.heading, EPS)
    }

    @Test
    fun lookupInterpolatesHeadingAcrossWrapByShortestAngle() {
        val history = PoseHistory(capacity = 4)
        history.add(0L, Pose(0.0, 0.0, Math.toRadians(350.0)))
        history.add(100L, Pose(0.0, 0.0, Math.toRadians(10.0)))

        val pose = history.lookup(50L)!!

        assertEquals(0.0, pose.heading, EPS)
    }

    @Test
    fun lookupReturnsNullOutsideBufferedWindow() {
        val history = PoseHistory(capacity = 2)
        history.add(0L, Pose(0.0, 0.0, 0.0))
        history.add(10L, Pose(1.0, 0.0, 0.0))
        history.add(20L, Pose(2.0, 0.0, 0.0))

        assertNull(history.lookup(0L))
        assertNull(history.lookup(30L))
    }

    private companion object {
        const val EPS = 1e-6
    }
}
