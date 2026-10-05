package org.firstinspires.ftc.teamcode.vision.hive

import kotlin.math.PI
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightPose
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Before
import org.junit.Test

class GoalGeometryTest {
    private val savedConfig = ConfigSnapshot(HiveConfig)

    @Before
    fun setUp() {
        HiveConfig.cameraHeightIn = 10.0
    }

    @After
    fun tearDown() = savedConfig.restore()

    @Test
    fun levelCameraMapsLensAxesOntoRobotAxes() {
        assertVec(Vec3(100.0, 0.0, 10.0), GoalGeometry.cameraToRobot(Vec3(0.0, 0.0, 100.0), 0.0))
        assertVec(Vec3(100.0, -10.0, 5.0), GoalGeometry.cameraToRobot(Vec3(10.0, 5.0, 100.0), 0.0))
    }

    @Test
    fun pitchedCameraLooksUp() {
        HiveConfig.cameraPitchUpDeg = 30.0
        assertVec(Vec3(86.60254, 0.0, 60.0), GoalGeometry.cameraToRobot(Vec3(0.0, 0.0, 100.0), 0.0))
    }

    @Test
    fun turretAngleRotatesAboutItsAxis() {
        HiveConfig.axisForwardIn = -3.0
        HiveConfig.axisLeftIn = 1.0
        HiveConfig.cameraForwardIn = 2.0
        val p = GoalGeometry.cameraToRobot(Vec3(0.0, 0.0, 50.0), PI / 2)
        assertVec(Vec3(-3.0, 1.0 + 52.0, 10.0), p)
        assertEquals(PI / 2, GoalGeometry.turretBearingRad(p), 1e-9)
        assertEquals(52.0, GoalGeometry.horizontalDistanceFromTurretIn(p), 1e-9)
    }

    @Test
    fun yawedCameraBearingIncludesTheYaw() {
        HiveConfig.cameraYawDeg = 10.0
        val p = GoalGeometry.cameraToRobot(Vec3(0.0, 0.0, 80.0), 0.25)
        assertEquals(0.25 + Math.toRadians(10.0), GoalGeometry.turretBearingRad(p), 1e-9)
    }

    @Test
    fun squarelyFacingTagPointsBehindAndAboveItsFace() {
        val tag = BiobuzzAprilTags.lookup(30)!!
        val pose = LimelightPose(0.0, 0.0, 2.0, 0.0, 0.0, 0.0)
        val goal = GoalGeometry.goalInCamera(pose, tag)
        assertVec(Vec3(6.5, -7.1874, 2.0 / 0.0254 + 5.622), goal)
        assertVec(Vec3(0.0, 0.0, -1.0), GoalGeometry.tagFacingInCamera(pose))
    }

    @Test
    fun everyTagOfARotatedClusterAgreesOnTheGoal() {
        val origin = Vec3(-8.0, -30.0, 70.0)
        val angles = Triple(-35.0, 12.0, 4.0)
        val probe = LimelightPose(0.0, 0.0, 0.0, angles.first, angles.second, angles.third)
        val rotation = probe.tagRotation()
        for (tag in BiobuzzAprilTags.tagsOf(BiobuzzAprilTags.lookup(38)!!.cell)) {
            val centre = origin + rotation.apply(
                Vec3(tag.offsetFromClusterCenterInches, BiobuzzAprilTags.TAG_ROW_Y_INCHES, BiobuzzAprilTags.TAG_ROW_Z_INCHES),
            )
            val m = centre * 0.0254
            val pose = LimelightPose(m.x, m.y, m.z, angles.first, angles.second, angles.third)
            assertVec(origin, GoalGeometry.goalInCamera(pose, tag))
        }
    }

    @Test
    fun facingDirectionIsMeasuredAgainstHorizontal() {
        assertEquals(90.0, GoalGeometry.degreesBelowHorizontal(Vec3(0.0, 0.0, -1.0)), 1e-9)
        assertEquals(60.0, GoalGeometry.degreesBelowHorizontal(Vec3(0.5, 0.0, -0.8660254)), 1e-6)
    }

    private fun assertVec(expected: Vec3, actual: Vec3) {
        assertEquals("x", expected.x, actual.x, 1e-4)
        assertEquals("y", expected.y, actual.y, 1e-4)
        assertEquals("z", expected.z, actual.z, 1e-4)
    }
}
