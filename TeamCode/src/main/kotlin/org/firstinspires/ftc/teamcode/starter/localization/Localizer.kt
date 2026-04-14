package org.firstinspires.ftc.teamcode.starter.localization

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.starter.core.SubsystemBase

/**
 * Read-only façade over the [Follower]'s internal localizer.
 *
 * Pedro's Follower already owns the real localizer (Pinpoint, OTOS, or three
 * wheels — configured in [org.firstinspires.ftc.teamcode.pedroPathing.Constants]).
 * This subsystem exists so higher-level code can query pose/velocity and
 * inject corrections without reaching into follower internals — and so that
 * `Ivy`-scheduled commands can declare a localisation requirement:
 *
 * ```kotlin
 * Command.build()
 *     .requiring(robot.localizer)
 *     .setExecute { /* consume pose */ }
 * ```
 *
 * Pose writes via [setPose] are always taken in Pedro coordinates.
 */
class Localizer(private val follower: Follower) : SubsystemBase("Localizer") {

    override fun init(hardwareMap: HardwareMap) {
        // Follower owns the Pinpoint / OTOS / odometry wheels; nothing to init here.
    }

    val pose: Pose get() = follower.pose
    val velocity: Pose get() = follower.velocity

    fun setPose(p: Pose) {
        follower.pose = p
    }

    fun setStartingPose(p: Pose) {
        follower.setStartingPose(p)
    }
}
