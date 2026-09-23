package org.firstinspires.ftc.teamcode.vision.hive

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightPose
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags

/** A point or direction in inches; the frame is named by whoever holds it. */
data class Vec3(val x: Double, val y: Double, val z: Double) {
    operator fun plus(o: Vec3) = Vec3(x + o.x, y + o.y, z + o.z)
    operator fun minus(o: Vec3) = Vec3(x - o.x, y - o.y, z - o.z)
    operator fun times(s: Double) = Vec3(x * s, y * s, z * s)

    val norm: Double get() = sqrt(x * x + y * y + z * z)

    fun distanceTo(o: Vec3): Double = (this - o).norm

    fun isFinite(): Boolean = x.isFinite() && y.isFinite() && z.isFinite()
}

/** Row-major 3×3 rotation matrix. */
internal class Rotation3(private val m: DoubleArray) {

    fun apply(v: Vec3) = Vec3(
        m[0] * v.x + m[1] * v.y + m[2] * v.z,
        m[3] * v.x + m[4] * v.y + m[5] * v.z,
        m[6] * v.x + m[7] * v.y + m[8] * v.z,
    )

    operator fun times(o: Rotation3): Rotation3 {
        val r = DoubleArray(9)
        for (i in 0 until 3) for (j in 0 until 3) {
            r[i * 3 + j] = m[i * 3] * o.m[j] + m[i * 3 + 1] * o.m[3 + j] + m[i * 3 + 2] * o.m[6 + j]
        }
        return Rotation3(r)
    }

    companion object {
        fun aboutX(rad: Double): Rotation3 {
            val c = cos(rad); val s = sin(rad)
            return Rotation3(doubleArrayOf(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c))
        }

        fun aboutY(rad: Double): Rotation3 {
            val c = cos(rad); val s = sin(rad)
            return Rotation3(doubleArrayOf(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c))
        }

        fun aboutZ(rad: Double): Rotation3 {
            val c = cos(rad); val s = sin(rad)
            return Rotation3(doubleArrayOf(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0))
        }
    }
}

private const val INCHES_PER_METER = 1.0 / 0.0254

/** Tag centre in Limelight camera space (+X right, +Y down, +Z out of the lens), inches. */
internal fun LimelightPose.positionInches() =
    Vec3(xMeters * INCHES_PER_METER, yMeters * INCHES_PER_METER, zMeters * INCHES_PER_METER)

/**
 * Orientation of the tag frame (+X right looking at the tag, +Y towards its
 * label band, +Z into the face) in Limelight camera space. The one place the
 * Euler convention lives.
 *
 * SDK 11.2.1 builds the Pose3D from the JSON `t6t_cs` array as roll = a[3],
 * pitch = a[4], yaw = a[5], which rotate about camera X, Y and Z. Assumed:
 * a tag squarely facing the lens reports zero rotation, and the angles compose
 * extrinsically X, then Y, then Z (WPILib `Rotation3d`, which Limelight uses).
 * Unverified on hardware; the Hive Tag Survey checks it (four tags of one
 * cluster agree on the goal only if this is right).
 */
internal fun LimelightPose.tagRotation(): Rotation3 =
    Rotation3.aboutZ(Math.toRadians(yawDegrees)) *
        Rotation3.aboutY(Math.toRadians(pitchDegrees)) *
        Rotation3.aboutX(Math.toRadians(rollDegrees))

/**
 * Pure geometry from a Limelight tag pose to the goal in robot coordinates.
 * Robot frame: x forward, y left, z up from the tiles, inches; see
 * [TurretCameraMountConfig] for the turret and lens conventions.
 */
object GoalGeometry {

    /** The CELL opening centre that [tag]'s solved pose points at, in Limelight camera space, inches. */
    fun goalInCamera(pose: LimelightPose, tag: BiobuzzAprilTags.Tag): Vec3 =
        pose.positionInches() + pose.tagRotation().apply(Vec3(tag.goalXInTagInches, tag.goalYInTagInches, tag.goalZInTagInches))

    /** Unit vector out of the tag face (towards whoever sees it), in Limelight camera space. */
    fun tagFacingInCamera(pose: LimelightPose): Vec3 = pose.tagRotation().apply(Vec3(0.0, 0.0, -1.0))

    /** A camera-space point, inches, in the robot frame at [turretAngleRad]. */
    fun cameraToRobot(point: Vec3, turretAngleRad: Double, mount: TurretCameraMount): Vec3 {
        val turret = rotateCameraToTurret(point, mount) + Vec3(mount.forwardIn, mount.leftIn, mount.heightIn)
        return rotateTurretToRobot(turret, turretAngleRad) + Vec3(mount.axisForwardIn, mount.axisLeftIn, 0.0)
    }

    /** A camera-space direction in the robot frame at [turretAngleRad]. */
    fun directionCameraToRobot(direction: Vec3, turretAngleRad: Double, mount: TurretCameraMount): Vec3 =
        rotateTurretToRobot(rotateCameraToTurret(direction, mount), turretAngleRad)

    /** Turret angle that points the turret at [pointRobot], radians, CCW from the robot's front. */
    fun turretBearingRad(pointRobot: Vec3, mount: TurretCameraMount): Double =
        atan2(pointRobot.y - mount.axisLeftIn, pointRobot.x - mount.axisForwardIn)

    /** Horizontal distance from the turret axis to [pointRobot], inches. */
    fun horizontalDistanceFromTurretIn(pointRobot: Vec3, mount: TurretCameraMount): Double =
        hypot(pointRobot.x - mount.axisForwardIn, pointRobot.y - mount.axisLeftIn)

    /** Bearing of [pointRobot] from the robot's pose point, radians, CCW from the robot's front. */
    fun robotBearingRad(pointRobot: Vec3): Double = atan2(pointRobot.y, pointRobot.x)

    /** Degrees a direction points below horizontal (90 = straight down). */
    fun degreesBelowHorizontal(direction: Vec3): Double =
        Math.toDegrees(atan2(-direction.z, hypot(direction.x, direction.y)))

    /** Camera (right, down, out of lens) → turret-aligned (forward, left, up), pitched then yawed. */
    private fun rotateCameraToTurret(v: Vec3, mount: TurretCameraMount): Vec3 {
        val forward = v.z
        val left = -v.x
        val up = -v.y
        val pitch = Math.toRadians(mount.pitchUpDeg)
        val f1 = forward * cos(pitch) - up * sin(pitch)
        val u1 = forward * sin(pitch) + up * cos(pitch)
        val yaw = Math.toRadians(mount.yawDeg)
        return Vec3(f1 * cos(yaw) - left * sin(yaw), f1 * sin(yaw) + left * cos(yaw), u1)
    }

    private fun rotateTurretToRobot(v: Vec3, turretAngleRad: Double): Vec3 {
        val c = cos(turretAngleRad)
        val s = sin(turretAngleRad)
        return Vec3(v.x * c - v.y * s, v.x * s + v.y * c, v.z)
    }
}
