package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.runtime.PersistedPose
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.util.Clock
import org.firstinspires.ftc.teamcode.core.util.composePose
import org.firstinspires.ftc.teamcode.core.util.relativePose

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
 *     .requiring(localizer)
 *     .setExecute { /* consume pose */ }
 * ```
 *
 * Pose writes via [setPose] are always taken in Pedro coordinates.
 */
class LocalizerSubsystem(
    private val follower: Follower,
    private val clock: Clock = Clock.SYSTEM,
) : SubsystemBase("Localizer") {

    private val history = PoseHistory()

    override fun init(hardwareMap: HardwareMap) {
        // Follower owns the Pinpoint / OTOS / odometry wheels; nothing to init here.
    }

    override fun periodic() {
        history.add(clock.nanos(), follower.pose)
    }

    val pose: Pose get() = follower.pose
    val velocity: Vector get() = follower.velocity

    fun setPose(p: Pose) {
        follower.pose = p
    }

    fun setStartingPose(p: Pose) {
        follower.setStartingPose(p)
    }

    /**
     * Restore a recently persisted field pose, usually from auton into teleop.
     *
     * @return true if a valid, fresh pose was applied to the follower.
     */
    fun restorePersistedPose(maxAgeMs: Long = 120_000): Boolean {
        if (!PersistedPose.valid) return false
        val ageMs = System.currentTimeMillis() - PersistedPose.wallTimeMs
        if (ageMs < 0 || ageMs > maxAgeMs) return false
        follower.setPose(Pose(PersistedPose.x, PersistedPose.y, PersistedPose.headingRad))
        return true
    }

    /**
     * Apply a delayed field-pose measurement while preserving motion since
     * [timestampNanos]. Returns false if the timestamp is stale or outside the
     * retained history window.
     */
    fun applyCorrection(
        measured: Pose,
        timestampNanos: Long,
        maxAgeNanos: Long = 500_000_000,
    ): Boolean {
        val age = clock.nanos() - timestampNanos
        if (age < 0 || age > maxAgeNanos) return false
        val historical = history.lookup(timestampNanos) ?: return false
        val current = follower.pose
        val relative = relativePose(historical, current)
        follower.setPose(composePose(measured, relative))
        return true
    }
}
