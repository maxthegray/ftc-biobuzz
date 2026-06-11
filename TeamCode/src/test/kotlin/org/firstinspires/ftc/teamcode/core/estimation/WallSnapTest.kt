package org.firstinspires.ftc.teamcode.core.estimation

import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.junit.Assert.assertEquals
import org.junit.Test

class WallSnapTest {

    private val eps = 1e-9
    private val fieldLength = 141.5

    @Test
    fun snapsThePerpendicularAxisAndCopiesTheParallelOne() {
        val current = Pose2d(3.0, 47.0, 0.02)

        val xMin = WallSnap.pose(WallSnap.Wall.X_MIN, 8.5, current, fieldLength)
        assertEquals(8.5, xMin.x, eps)
        assertEquals(47.0, xMin.y, eps)

        val xMax = WallSnap.pose(WallSnap.Wall.X_MAX, 8.5, current, fieldLength)
        assertEquals(fieldLength - 8.5, xMax.x, eps)
        assertEquals(47.0, xMax.y, eps)

        val yMin = WallSnap.pose(WallSnap.Wall.Y_MIN, 7.0, current, fieldLength)
        assertEquals(3.0, yMin.x, eps)
        assertEquals(7.0, yMin.y, eps)

        val yMax = WallSnap.pose(WallSnap.Wall.Y_MAX, 7.0, current, fieldLength)
        assertEquals(3.0, yMax.x, eps)
        assertEquals(fieldLength - 7.0, yMax.y, eps)
    }

    @Test
    fun snapsHeadingToTheNearestCardinal() {
        val nearZero = Pose2d(0.0, 0.0, Math.toRadians(8.0))
        assertEquals(0.0, WallSnap.pose(WallSnap.Wall.X_MIN, 8.0, nearZero).heading, eps)

        val near90 = Pose2d(0.0, 0.0, Math.toRadians(95.0))
        assertEquals(Math.PI / 2.0, WallSnap.pose(WallSnap.Wall.X_MIN, 8.0, near90).heading, eps)

        // Wraparound: 355° snaps to 0°, not 360°.
        val near360 = Pose2d(0.0, 0.0, Math.toRadians(355.0))
        assertEquals(0.0, WallSnap.pose(WallSnap.Wall.X_MIN, 8.0, near360).heading, eps)
    }

    @Test
    fun headingSnapCanBeDisabled() {
        val current = Pose2d(0.0, 0.0, 0.3)
        val pose = WallSnap.pose(WallSnap.Wall.Y_MIN, 8.0, current, snapHeading = false)
        assertEquals(0.3, pose.heading, eps)
    }

    @Test
    fun nearestCardinalCoversAllQuadrants() {
        assertEquals(Math.PI, WallSnap.nearestCardinal(Math.toRadians(170.0)), eps)
        assertEquals(1.5 * Math.PI, WallSnap.nearestCardinal(Math.toRadians(280.0)), eps)
        assertEquals(0.0, WallSnap.nearestCardinal(Math.toRadians(-20.0)), eps)
    }
}
