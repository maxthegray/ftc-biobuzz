package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.estimation.CorrectionResult
import org.firstinspires.ftc.teamcode.core.estimation.PoseEstimator
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.Vector2d
import org.firstinspires.ftc.teamcode.core.pathing.toCore
import org.firstinspires.ftc.teamcode.core.pathing.toPedro
import org.firstinspires.ftc.teamcode.core.runtime.PersistedPose
import org.firstinspires.ftc.teamcode.core.runtime.PoseProvider
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * Read-only façade over the [Follower]'s internal localizer, plus the
 * external-correction seam (vision, wall snaps) via [PoseEstimator].
 *
 * Pedro's Follower already owns the real localizer (Pinpoint, OTOS, or three
 * wheels — configured in [org.firstinspires.ftc.teamcode.pedroPathing.Constants]).
 * This subsystem exists so higher-level code can query pose/velocity and
 * inject corrections without reaching into follower internals — and so that
 * scheduler commands can declare a localisation requirement.
 *
 * **Registration order matters:** register this *after* the drive subsystem
 * (enforced by [registerAfter]). The pose history is sampled in
 * [writeHardware], immediately after `MecanumDriveSubsystem.writeHardware()`
 * runs `Follower.update()` — so each sample carries the timestamp the pose
 * was actually measured. Sampling in `periodic()` would timestamp the
 * *previous* tick's pose with this tick's clock, skewing every
 * latency-compensated correction by one loop period.
 *
 * Wire [isFollowing] (typically `drive::isFollowing`) and accepted
 * corrections are scaled by [LocalizerConfig.followingBlendScale] while a
 * path is running — see [PoseEstimator] for why.
 */
class LocalizerSubsystem(
    private val follower: Follower,
    private val clock: Clock = Clock.SYSTEM,
    private val onEvent: (String) -> Unit = {},
    isFollowing: () -> Boolean = { false },
) : SubsystemBase("Localizer"), PoseProvider {

    val estimator = PoseEstimator(
        currentPose = { pose },
        applyPose = { setPose(it) },
        clock = clock,
        onEvent = onEvent,
        isFollowing = isFollowing,
    )

    /** Enforced by Robot.register — see the class doc's registration-order contract. */
    override val registerAfter: Class<out SubsystemBase>
        get() = org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem::class.java

    override fun init(hardwareMap: HardwareMap) {
        // Follower owns the Pinpoint / OTOS / odometry wheels; nothing to init here.
    }

    override fun writeHardware() {
        // Not a hardware write — this runs here (after the drive subsystem's
        // Follower.update()) so the sample timestamp matches when the pose
        // was measured. See the class doc.
        estimator.sample(clock.nanos(), pose)
    }

    override val pose: Pose2d get() = follower.pose.toCore()
    override val velocity: Vector2d get() = follower.velocity.toCore()

    fun setPose(p: Pose2d) {
        follower.pose = p.toPedro()
    }

    fun setStartingPose(p: Pose2d) {
        follower.setStartingPose(p.toPedro())
    }

    /**
     * Restore a recently persisted field pose, usually from auton into teleop.
     * Falls back to the on-disk copy when the Robot Controller process
     * restarted between op-modes.
     *
     * @return true if a valid, fresh pose was applied to the follower.
     */
    fun restorePersistedPose(maxAgeMs: Long = 120_000): Boolean {
        PersistedPose.restoreFromDiskIfNeeded()
        if (!PersistedPose.valid) return false
        val ageMs = System.currentTimeMillis() - PersistedPose.wallTimeMs
        if (ageMs < 0 || ageMs > maxAgeMs) return false
        setPose(Pose2d(PersistedPose.x, PersistedPose.y, PersistedPose.headingRad))
        return true
    }

    /**
     * Apply a delayed field-pose measurement while preserving motion since
     * [timestampNanos] — see [PoseEstimator.applyCorrection] for gating,
     * blending, axis weights, and the during-follow policy.
     */
    fun applyCorrection(
        measured: Pose2d,
        timestampNanos: Long,
        maxAgeNanos: Long = 500_000_000,
        blend: Double = LocalizerConfig.safeCorrectionBlend,
        maxJumpInches: Double = LocalizerConfig.safeMaxCorrectionInches,
        maxJumpRadians: Double = LocalizerConfig.safeMaxCorrectionRadians,
        translationWeight: Double = 1.0,
        headingWeight: Double = 1.0,
    ): CorrectionResult = estimator.applyCorrection(
        measured = measured,
        timestampNanos = timestampNanos,
        maxAgeNanos = maxAgeNanos,
        blend = blend,
        maxJumpInches = maxJumpInches,
        maxJumpRadians = maxJumpRadians,
        translationWeight = translationWeight,
        headingWeight = headingWeight,
    )
}
