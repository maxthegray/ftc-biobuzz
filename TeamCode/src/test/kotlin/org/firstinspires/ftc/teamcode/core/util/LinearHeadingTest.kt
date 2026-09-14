package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.api.Paths
import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.pedropathing.utils.Angle
import kotlin.math.PI
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.drive.PedroDriveFixture
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class LinearHeadingTest {
    private val a = Pose(0.0, 0.0, 0.0)
    private val b = Pose(48.0, 0.0, PI / 2)

    private fun assertHeading(expected: Double, actual: Double) =
        assertEquals(0.0, Angle.error(expected, actual), 1e-6)

    private fun assertStartMidEnd(path: Path, start: Double, mid: Double, end: Double) {
        assertHeading(start, path.heading(0.0))
        assertHeading(mid, path.heading(0.5))
        assertHeading(end, path.heading(1.0))
    }

    @Test
    fun lineTurnsFromStartToEnd() {
        assertStartMidEnd(Paths.line(a, b).heading(linearHeading(a, b)), 0.0, PI / 4, PI / 2)
        // Pedro's own linear on the same line, for contrast.
        assertStartMidEnd(Paths.line(a, b).linear(a, b), PI / 2, PI / 4, 0.0)
    }

    @Test
    fun curveMatchesPedrosCorrectBezierInterpolation() {
        val curve = Paths.curve(a, Pose(10.0, 30.0), b)
        val ours = curve.heading(linearHeading(a, b))
        val pedro = curve.linear(a, b)
        for (t in listOf(0.0, 0.1, 0.37, 0.5, 0.8, 1.0)) {
            assertHeading(pedro.heading(t), ours.heading(t))
        }
    }

    @Test
    fun compoundPathInterpolatesByDistanceAcrossSegments() {
        // 24 in then 72 in: the join is a quarter of the way along.
        val m = Pose(24.0, 0.0)
        val end = Pose(24.0, 72.0, PI / 2)
        val path = Paths.path(Paths.line(a, m).constant(0.0), Paths.line(m, end).constant(0.0))
            .heading(linearHeading(a, end))

        assertHeading(0.0, path.heading(0.0))
        assertHeading(PI / 8, path.heading(0.25))
        assertHeading(PI / 4, path.heading(0.5))
        assertHeading(PI / 2, path.heading(1.0))

        // What the follower actually tracks: the per-segment headings join up.
        val segments = path.segments
        assertEquals(2, segments.size)
        assertHeading(0.0, segments[0].heading(0.0))
        assertHeading(PI / 8, segments[0].heading(1.0))
        assertHeading(PI / 8, segments[1].heading(0.0))
        assertHeading(PI / 2, segments[1].heading(1.0))
        assertHeading(PI / 2, path.endPose().heading())

        // Pedro's linear on the same compound path runs backwards.
        val pedro = Paths.path(Paths.line(a, m).constant(0.0), Paths.line(m, end).constant(0.0)).linear(a, end)
        assertHeading(PI / 2, pedro.heading(0.0))
        assertHeading(0.0, pedro.heading(1.0))
    }

    @Test
    fun wrapsTheShortWayAndStaysNormalized() {
        val start = Pose(0.0, 0.0, Math.toRadians(350.0))
        val finish = Pose(48.0, 0.0, Math.toRadians(10.0))
        val path = Paths.line(start, finish).heading(linearHeading(start, finish))
        assertStartMidEnd(path, Math.toRadians(350.0), 0.0, Math.toRadians(10.0))
        for (i in 0..20) {
            val h = path.heading(i / 20.0)
            assertTrue(h >= 0.0 && h < 2 * PI)
            assertTrue(abs(Angle.error(0.0, h)) <= Math.toRadians(10.0) + 1e-9)
        }
    }

    @Test
    fun blueFollowsTheMirroredHeadingsAtEveryPoint() {
        val red = Paths.line(a, b).heading(linearHeading(a, b))
        for (symmetry in FieldSymmetry.entries) {
            val blueA = Alliance.BLUE.mirror(a, symmetry)
            val blueB = Alliance.BLUE.mirror(b, symmetry)
            val blue = Paths.line(blueA, blueB).heading(linearHeading(blueA, blueB))
            for (t in listOf(0.0, 0.25, 0.5, 0.75, 1.0)) {
                assertHeading(Alliance.BLUE.mirror(red.heading(t), symmetry), blue.heading(t))
            }
        }
    }

    @Test
    fun exampleAutoReturnLegTurnsTheRightWayOnBothAlliances() {
        // ExampleAuto.backPath(): out (32, 56) facing 90° back to start (8, 56) facing 0°, in RED coordinates.
        for (alliance in Alliance.entries) {
            val poses = alliance.poses()
            val outTurned = poses.of(32.0, 56.0, 90.0)
            val start = poses.of(8.0, 56.0, 0.0)
            val back = Paths.line(outTurned, start).heading(linearHeading(outTurned, start))
            assertStartMidEnd(back, outTurned.heading(), alliance.mirror(PI / 4), start.heading())
            assertHeading(start.heading(), back.endPose().heading())
        }
    }

    @Test
    fun theFollowerTargetsTheStartHeadingAtTheStart() {
        val start = Pose(0.0, 0.0, PI / 2)
        val finish = Pose(48.0, 0.0, 0.0)
        fun headingErrorAtStart(path: Path): Double {
            Scheduler.reset()
            val hardware = PedroDriveFixture()
            val robot = Robot(hardware.hardwareMap, hardware.clock)
            robot.register(hardware.drive)
            robot.start()
            hardware.localizer.measuredPose = start
            Scheduler.schedule(hardware.drive.followCommand(path))
            hardware.clock.advanceMs(20.0)
            robot.loop()
            return abs(hardware.drive.followHeadingErrorRad)
        }
        assertEquals(0.0, headingErrorAtStart(Paths.line(start, finish).heading(linearHeading(start, finish))), 1e-3)
        assertEquals(PI / 2, headingErrorAtStart(Paths.line(start, finish).linear(start, finish)), 1e-3)
    }
}
