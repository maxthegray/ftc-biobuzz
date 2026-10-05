package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.StateLog

/**
 * Geared position servos; manual and camera aiming will share this hardware owner.
 *
 * The Limelight rides on the turret. Once hardware exists, read the servo
 * encoder every tick in `periodic()`, convert it to turret radians (gear ratio
 * included, CCW from the robot's front), keep a short timestamped history of
 * it (`clock.nanos()`, interpolated like `PoseHistory`, no angle wrapping), and
 * pass a lookup `(nanos) -> Double?` to `HiveTracker` as `turretAngleAt`.
 * Aim at `hive.aimGoal(alliance).turretBearingRad`: the vision goal, or the
 * odometry estimate until tags are seen. Shoot only when its source is VISION.
 */
class TurretSubsystem : SubsystemBase("Turret") {
    override fun init(hardwareMap: HardwareMap) {
        error("Turret hardware is not configured yet")
    }

    override fun health(): String = "Hardware not configured"

    override fun logState(log: StateLog) {
        log.put("configured", false)
    }
}
