package org.firstinspires.ftc.teamcode.core.geometry

import com.pedropathing.geometry.Pose
import kotlin.math.PI
import kotlin.random.Random
import org.junit.Assert.assertEquals
import org.junit.Test

class Pose2dTest {

    private val eps = 1e-9

    // ----------------------------------------------------------------- angles

    @Test
    fun normalizeAngleWrapsInto0To2Pi() {
        assertEquals(0.0, normalizeAngle(0.0), eps)
        assertEquals(0.0, normalizeAngle(2.0 * PI), eps)
        assertEquals(PI, normalizeAngle(-PI), eps)
        assertEquals(1.75 * PI, normalizeAngle(-0.25 * PI), eps)
        assertEquals(0.5 * PI, normalizeAngle(4.5 * PI), eps)
    }

    @Test
    fun normalizeAngleSignedWrapsIntoMinusPiToPi() {
        assertEquals(0.0, normalizeAngleSigned(2.0 * PI), eps)
        assertEquals(-0.5 * PI, normalizeAngleSigned(1.5 * PI), eps)
        assertEquals(PI, normalizeAngleSigned(PI), eps)
    }

    @Test
    fun shortestAngleDeltaTakesTheShortArcAndResolvesHalfTurnPositive() {
        assertEquals(Math.toRadians(20.0), shortestAngleDelta(Math.toRadians(350.0), Math.toRadians(10.0)), eps)
        assertEquals(-Math.toRadians(20.0), shortestAngleDelta(Math.toRadians(10.0), Math.toRadians(350.0)), eps)
        // Exact half-turn resolves to +π, never -π.
        assertEquals(PI, shortestAngleDelta(0.0, PI), eps)
        assertEquals(PI, shortestAngleDelta(PI, 0.0), eps)
    }

    // -------------------------------------------------- relative / transform

    @Test
    fun relativeToExpressesAPoseInAnotherFrame() {
        val origin = Pose2d(10.0, 0.0, PI / 2.0)
        val target = Pose2d(10.0, 5.0, PI)
        val relative = target.relativeTo(origin)
        // 5 inches ahead of a pose facing +y is +x in its frame.
        assertEquals(5.0, relative.x, eps)
        assertEquals(0.0, relative.y, eps)
        assertEquals(PI / 2.0, relative.heading, eps)
    }

    @Test
    fun transformByIsTheInverseOfRelativeTo() {
        val random = Random(42)
        repeat(200) {
            val a = randomPose(random)
            val b = randomPose(random)
            val recomposed = a.transformBy(b.relativeTo(a))
            assertEquals(b.x, recomposed.x, 1e-9)
            assertEquals(b.y, recomposed.y, 1e-9)
            assertEquals(0.0, normalizeAngleSigned(b.heading - recomposed.heading), 1e-9)
        }
    }

    @Test
    fun lerpInterpolatesHeadingAcrossTheWrap() {
        val a = Pose2d(0.0, 0.0, Math.toRadians(350.0))
        val b = Pose2d(10.0, 20.0, Math.toRadians(10.0))
        val mid = a.lerp(b, 0.5)
        assertEquals(5.0, mid.x, eps)
        assertEquals(10.0, mid.y, eps)
        assertEquals(0.0, mid.heading, eps)
    }

    // ----------------------------------------------------------------- mirror

    @Test
    fun mirrorParityWithPedroPoseMirror() {
        val random = Random(7)
        val fieldLength = 141.5
        repeat(200) {
            val pose = randomPose(random)
            val ours = pose.mirror(FieldSymmetry.MIRROR, fieldLength)
            val pedro = Pose(pose.x, pose.y, pose.heading).mirror(fieldLength)
            assertEquals(pedro.x, ours.x, 1e-9)
            assertEquals(pedro.y, ours.y, 1e-9)
            assertEquals(0.0, normalizeAngleSigned(pedro.heading - ours.heading), 1e-9)
        }
    }

    @Test
    fun rotateMirrorIsA180RotationAboutFieldCenter() {
        val pose = Pose2d(10.0, 30.0, 0.5)
        val rotated = pose.mirror(FieldSymmetry.ROTATE, 141.5)
        assertEquals(131.5, rotated.x, eps)
        assertEquals(111.5, rotated.y, eps)
        assertEquals(normalizeAngle(0.5 + PI), rotated.heading, eps)
    }

    // ----------------------------------------------------------------- vector

    @Test
    fun vectorRotationAndMagnitude() {
        val v = Vector2d(3.0, 4.0)
        assertEquals(5.0, v.magnitude, eps)
        val rotated = v.rotateBy(PI / 2.0)
        assertEquals(-4.0, rotated.x, eps)
        assertEquals(3.0, rotated.y, eps)
    }

    private fun randomPose(random: Random): Pose2d = Pose2d(
        random.nextDouble(-150.0, 150.0),
        random.nextDouble(-150.0, 150.0),
        random.nextDouble(-4.0 * PI, 4.0 * PI),
    )
}
