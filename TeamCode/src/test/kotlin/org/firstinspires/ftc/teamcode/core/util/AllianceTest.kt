package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig
import org.junit.Assert.assertEquals
import org.junit.Test

class AllianceTest {

    private val eps = 1e-9

    private fun assertPose(expected: Pose, actual: Pose) {
        assertEquals("x", expected.x, actual.x, eps)
        assertEquals("y", expected.y, actual.y, eps)
        assertEquals("heading", expected.heading, actual.heading, eps)
    }

    @Test
    fun redIsIdentity() {
        val p = Pose(12.0, 34.0, 1.25)
        assertPose(p, Alliance.RED.mirror(p))
    }

    @Test
    fun blueMirrorsAcrossConfiguredFieldLength() {
        val l = RobotConfig.Field.LENGTH_INCHES
        val p = Pose(10.0, 30.0, 0.0)
        // Pedro's Pose.mirror(L) = (L - x, y, normalize(pi - heading)).
        assertPose(Pose(l - 10.0, 30.0, Math.PI), Alliance.BLUE.mirror(p))
    }

    @Test
    fun blueMirrorNormalizesHeading() {
        val l = RobotConfig.Field.LENGTH_INCHES
        val mirrored = Alliance.BLUE.mirror(Pose(0.0, 0.0, Math.toRadians(-45.0)))
        assertEquals(l, mirrored.x, eps)
        assertEquals(Math.toRadians(225.0), mirrored.heading, 1e-9)
    }

    @Test
    fun blueMirrorIsAnInvolution() {
        val p = Pose(48.0, 96.0, 2.0)
        val twice = Alliance.BLUE.mirror(Alliance.BLUE.mirror(p))
        assertEquals(p.x, twice.x, eps)
        assertEquals(p.y, twice.y, eps)
        // Heading comes back normalized into [0, 2pi).
        assertEquals(p.heading, twice.heading, eps)
    }

    @Test
    fun rotationalSymmetryRotatesAboutFieldCenter() {
        val l = RobotConfig.Field.LENGTH_INCHES
        val p = Pose(10.0, 30.0, 0.0)
        val rotated = Alliance.BLUE.mirror(p, FieldSymmetry.ROTATE)
        assertPose(Pose(l - 10.0, l - 30.0, Math.PI), rotated)
    }

    @Test
    fun rotationalSymmetryIsAnInvolution() {
        val p = Pose(48.0, 96.0, 2.0)
        val twice = Alliance.BLUE.mirror(Alliance.BLUE.mirror(p, FieldSymmetry.ROTATE), FieldSymmetry.ROTATE)
        assertEquals(p.x, twice.x, eps)
        assertEquals(p.y, twice.y, eps)
        assertEquals(p.heading, twice.heading, eps)
    }

    @Test
    fun rotationalHeadingMirrorAddsHalfTurn() {
        assertEquals(
            Math.PI,
            Alliance.BLUE.mirror(0.0, FieldSymmetry.ROTATE),
            eps,
        )
        // RED is always identity regardless of symmetry.
        assertEquals(1.25, Alliance.RED.mirror(1.25, FieldSymmetry.ROTATE), eps)
    }

    @Test
    fun headingMirrorMatchesPoseMirrorHeadingForBothSymmetries() {
        val p = Pose(20.0, 40.0, 0.7)
        for (symmetry in FieldSymmetry.entries) {
            val poseHeading = Alliance.BLUE.mirror(p, symmetry).heading
            val bareHeading = Alliance.BLUE.mirror(p.heading, symmetry)
            assertEquals(
                "symmetry $symmetry",
                0.0,
                com.pedropathing.math.MathFunctions.normalizeAngleSigned(poseHeading - bareHeading),
                eps,
            )
        }
    }
}
