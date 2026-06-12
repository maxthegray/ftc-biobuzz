package org.firstinspires.ftc.teamcode.core.pathing

import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.ftc.drivetrains.Mecanum
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.BezierPoint
import com.pedropathing.geometry.Curve
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.pedropathing.math.Vector
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathConstraints
import org.firstinspires.ftc.teamcode.core.geometry.FieldSymmetry
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.normalizeAngle
import org.firstinspires.ftc.teamcode.core.sim.SimFollower
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.fail
import org.junit.Test

/**
 * Pins the exact Pedro API surface the framework calls — the adapter layer
 * (`core/pathing/`, `MecanumDriveSubsystem`, `LocalizerSubsystem`, the
 * localizer implementations) plus the test-side [SimFollower].
 *
 * A Pedro version bump that renames or retypes any of these fails here with
 * a named member instead of surfacing as a runtime crash on the robot (or
 * worse, as silently changed behavior under a base-class member the sim
 * doesn't override). If this test fails after a bump, every listed member
 * is a call site to re-verify by hand.
 */
class PedroSurfaceContractTest {

    // ------------------------------------------------------------- reflection

    private val missing = mutableListOf<String>()

    private fun method(clazz: Class<*>, name: String, vararg params: Class<*>) {
        try {
            clazz.getMethod(name, *params)
        } catch (_: NoSuchMethodException) {
            missing += "${clazz.simpleName}.$name(${params.joinToString { it.simpleName }})"
        }
    }

    private fun constructor(clazz: Class<*>, vararg params: Class<*>) {
        try {
            clazz.getConstructor(*params)
        } catch (_: NoSuchMethodException) {
            missing += "${clazz.simpleName}(${params.joinToString { it.simpleName }})"
        }
    }

    private fun field(clazz: Class<*>, name: String) {
        try {
            clazz.getField(name)
        } catch (_: NoSuchFieldException) {
            missing += "${clazz.simpleName}.$name"
        }
    }

    private fun assertNothingMissing() {
        if (missing.isNotEmpty()) {
            fail("Pedro surface changed; re-verify these call sites:\n" + missing.joinToString("\n"))
        }
    }

    @Test
    fun followerSurfaceTheFrameworkCallsExists() {
        val d = java.lang.Double.TYPE
        val b = java.lang.Boolean.TYPE
        val f = Follower::class.java

        method(f, "update")
        method(f, "getPose")
        method(f, "setPose", Pose::class.java)
        method(f, "setStartingPose", Pose::class.java)
        method(f, "getVelocity")
        method(f, "startTeleOpDrive", b)
        method(f, "setTeleOpDrive", d, d, d, b)
        method(f, "followPath", PathChain::class.java, b)
        method(f, "followPath", PathChain::class.java, d, b)
        method(f, "followPath", Path::class.java)
        method(f, "holdPoint", Pose::class.java)
        method(f, "breakFollowing")
        method(f, "isBusy")
        method(f, "getTranslationalError")
        method(f, "getHeadingError")
        method(f, "getConstraints")
        // `follower.pathConstraints` resolves to a public field, not a getter.
        field(f, "pathConstraints")
        method(f, "getCurrentPathChain")
        method(f, "getCurrentPathNumber")
        method(f, "getCurrentTValue")
        method(f, "getCurrentPath")
        method(f, "getAngularVelocity")
        method(f, "pathBuilder")
        method(f, "getDrivetrain")

        assertNothingMissing()
    }

    @Test
    fun geometryAndPathSurfaceTheFrameworkCallsExists() {
        val d = java.lang.Double.TYPE

        method(PathChain::class.java, "size")
        method(PathChain::class.java, "getPath", Integer.TYPE)
        method(Path::class.java, "getPose", d)
        method(Path::class.java, "getHeadingGoal", d)
        method(Path::class.java, "getLastControlPoint")
        method(Path::class.java, "setHeadingInterpolation", HeadingInterpolator::class.java)
        method(Path::class.java, "setConstraints", PathConstraints::class.java)
        // Path(BezierPoint(...)) compiles via the Curve supertype.
        constructor(Path::class.java, Curve::class.java)
        constructor(BezierPoint::class.java, Pose::class.java)
        constructor(BezierLine::class.java, Pose::class.java, Pose::class.java)
        constructor(BezierCurve::class.java, List::class.java)
        method(HeadingInterpolator::class.java, "constant", d)
        method(Pose::class.java, "getX")
        method(Pose::class.java, "getY")
        method(Pose::class.java, "getHeading")
        method(Pose::class.java, "minus", Pose::class.java)
        method(Pose::class.java, "mirror", d)
        method(Vector::class.java, "getXComponent")
        method(Vector::class.java, "getYComponent")
        method(Vector::class.java, "getMagnitude")
        method(MathFunctions::class.java, "normalizeAngle", d)
        method(Mecanum::class.java, "getMotors")
        method(Drivetrain::class.java, "breakFollowing")

        assertNothingMissing()
    }

    // ------------------------------------------------------------- behavioral

    @Test
    fun poseConversionRoundTrips() {
        val original = Pose2d(12.5, -3.25, 2.1)
        val back = original.toPedro().toCore()
        assertEquals(original.x, back.x, 1e-12)
        assertEquals(original.y, back.y, 1e-12)
        assertEquals(original.heading, back.heading, 1e-12)
    }

    @Test
    fun mirrorMatchesPedroPoseMirror() {
        // Pose2d.mirror's kdoc claims MIRROR matches Pedro's Pose.mirror(length)
        // exactly — hold Pedro to it.
        val fieldLength = 141.5
        val core = Pose2d(10.0, 20.0, 0.5).mirror(FieldSymmetry.MIRROR, fieldLength)
        val pedro = Pose(10.0, 20.0, 0.5).mirror(fieldLength)
        assertEquals(pedro.x, core.x, 1e-12)
        assertEquals(pedro.y, core.y, 1e-12)
        assertEquals(normalizeAngle(pedro.heading), core.heading, 1e-12)
    }

    @Test
    fun pathBuilderBuildsChainsWithExpectedGeometry() {
        // Built through the same entry point the framework uses
        // (Follower.pathBuilder()), against the real PathBuilder/PathChain.
        val chain = SimFollower(FakeClock()).pathBuilder()
            .addPath(BezierLine(Pose(0.0, 0.0), Pose(24.0, 0.0)))
            .addPath(BezierLine(Pose(24.0, 0.0), Pose(24.0, 24.0)))
            .build()

        assertEquals(2, chain.size())
        val firstEnd = chain.getPath(0).getPose(1.0)
        assertEquals(24.0, firstEnd.x, 1e-9)
        assertEquals(0.0, firstEnd.y, 1e-9)
        val lastPoint = chain.getPath(1).lastControlPoint
        assertEquals(24.0, lastPoint.x, 1e-9)
        assertEquals(24.0, lastPoint.y, 1e-9)
    }
}
