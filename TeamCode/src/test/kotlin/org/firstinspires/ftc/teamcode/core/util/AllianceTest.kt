package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.math.Pose
import com.pedropathing.utils.Angle
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig
import org.junit.Assert.assertEquals
import org.junit.Test

class AllianceTest {

    private val eps = 1e-9

    private fun assertPose(expected: Pose, actual: Pose) {
        assertEquals("x", expected.x(), actual.x(), eps)
        assertEquals("y", expected.y(), actual.y(), eps)
        assertEquals("heading", expected.heading(), actual.heading(), eps)
    }

    @Test
    fun redIsIdentity() {
        val p = Pose(12.0, 34.0, 1.25)
        assertPose(p, Alliance.RED.mirror(p))
    }

    @Test
    fun blueMirrorsAcrossConfiguredFieldLengthLikePedro2PoseMirror() {
        val l = RobotConfig.Field.LENGTH_INCHES
        // MIRROR symmetry = (L - x, y, pi - heading), which Pedro 2.1.1's Pose.mirror(L) produced.
        assertPose(Pose(l - 10.0, 30.0, Math.PI), Alliance.BLUE.mirror(Pose(10.0, 30.0, 0.0)))
        assertPose(Pose(l - 10.0, 20.0, Math.PI - 0.5), Alliance.BLUE.mirror(Pose(10.0, 20.0, 0.5)))
    }

    @Test
    fun blueMirrorNormalizesHeading() {
        val mirrored = Alliance.BLUE.mirror(Pose(0.0, 0.0, Math.toRadians(-45.0)))
        assertEquals(RobotConfig.Field.LENGTH_INCHES, mirrored.x(), eps)
        assertEquals(Math.toRadians(225.0), mirrored.heading(), eps)
    }

    @Test
    fun mirrorsAreInvolutions() {
        val p = Pose(48.0, 96.0, 2.0)
        for (symmetry in FieldSymmetry.entries) {
            assertPose(p, Alliance.BLUE.mirror(Alliance.BLUE.mirror(p, symmetry), symmetry))
        }
    }

    @Test
    fun rotationalSymmetryRotatesAboutFieldCenter() {
        val l = RobotConfig.Field.LENGTH_INCHES
        assertPose(Pose(l - 10.0, l - 30.0, Math.PI), Alliance.BLUE.mirror(Pose(10.0, 30.0, 0.0), FieldSymmetry.ROTATE))
    }

    @Test
    fun headingMirrorMatchesPoseMirrorHeadingForBothSymmetries() {
        val p = Pose(20.0, 40.0, 0.7)
        for (symmetry in FieldSymmetry.entries) {
            val poseHeading = Alliance.BLUE.mirror(p, symmetry).heading()
            val bareHeading = Alliance.BLUE.mirror(p.heading(), symmetry)
            assertEquals("symmetry $symmetry", 0.0, Angle.normalizeSigned(poseHeading - bareHeading), eps)
        }
        assertEquals(1.25, Alliance.RED.mirror(1.25, FieldSymmetry.ROTATE), eps)
    }

    @Test
    fun poseFactoryTakesDegreesAndAppliesTheAllianceTransform() {
        val l = RobotConfig.Field.LENGTH_INCHES
        assertPose(Pose(8.0, 56.0, Math.PI / 2), Alliance.RED.poses().of(8.0, 56.0, 90.0))
        assertPose(Alliance.BLUE.mirror(Pose(8.0, 56.0, Math.PI / 2)), Alliance.BLUE.poses().of(8.0, 56.0, 90.0))
        assertEquals(l - 8.0, Alliance.BLUE.poses().of(8.0, 56.0, 90.0).x(), eps)
    }
}
