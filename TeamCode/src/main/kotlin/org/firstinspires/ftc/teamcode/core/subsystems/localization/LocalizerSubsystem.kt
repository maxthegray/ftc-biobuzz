package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Locale
import kotlin.math.abs
import kotlin.math.hypot
import org.firstinspires.ftc.teamcode.core.runtime.PersistedPose
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.util.Clock
import org.firstinspires.ftc.teamcode.core.util.composePose
import org.firstinspires.ftc.teamcode.core.util.relativePose
import org.firstinspires.ftc.teamcode.core.util.shortestHeadingDelta

/**
 * Read-only façade over the [Follower]'s internal localizer, plus the
 * external-correction seam (vision, wall snaps).
 *
 * Pedro's Follower already owns the real localizer (Pinpoint, OTOS, or three
 * wheels — configured in [org.firstinspires.ftc.teamcode.pedroPathing.Constants]).
 * This subsystem exists so higher-level code can query pose/velocity and
 * inject corrections without reaching into follower internals — and so that
 * `Ivy`-scheduled commands can declare a localisation requirement.
 *
 * **Registration order matters:** register this *after* the drive subsystem.
 * The pose history is sampled in [writeHardware], immediately after
 * `MecanumDriveSubsystem.writeHardware()` runs `Follower.update()` — so each
 * sample carries the timestamp the pose was actually measured. Sampling in
 * `periodic()` would timestamp the *previous* tick's pose with this tick's
 * clock, skewing every latency-compensated correction by one loop period.
 *
 * Pose writes via [setPose] are always taken in Pedro coordinates.
 */
class LocalizerSubsystem(
    private val follower: Follower,
    private val clock: Clock = Clock.SYSTEM,
    private val onEvent: (String) -> Unit = {},
) : SubsystemBase("Localizer") {

    /** Outcome of an [applyCorrection] attempt. Rejections are logged via the event sink. */
    enum class CorrectionResult {
        APPLIED,

        /** Measurement older than `maxAgeNanos` (or timestamped in the future). */
        STALE,

        /** Timestamp outside the retained pose-history window. */
        NO_HISTORY,

        /** Correction larger than the configured outlier gates. */
        REJECTED_JUMP;

        val accepted: Boolean get() = this == APPLIED
    }

    private val history = PoseHistory()

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
        follower.setPose(Pose(PersistedPose.x, PersistedPose.y, PersistedPose.headingRad))
        return true
    }

    /**
     * Apply a delayed field-pose measurement while preserving motion since
     * [timestampNanos]. The correction is gated (rejected as an outlier when
     * it exceeds [maxJumpInches] / [maxJumpRadians]) and blended ([blend]
     * fraction applied per call — see [LocalizerConfig.correctionBlend] for
     * how to choose it). Every rejection is recorded via the event sink so
     * the flight log shows why a correction didn't take.
     */
    fun applyCorrection(
        measured: Pose,
        timestampNanos: Long,
        maxAgeNanos: Long = 500_000_000,
        blend: Double = LocalizerConfig.safeCorrectionBlend,
        maxJumpInches: Double = LocalizerConfig.safeMaxCorrectionInches,
        maxJumpRadians: Double = LocalizerConfig.safeMaxCorrectionRadians,
    ): CorrectionResult {
        val age = clock.nanos() - timestampNanos
        if (age < 0 || age > maxAgeNanos) {
            onEvent("pose correction rejected: stale (age ${age / 1_000_000} ms)")
            return CorrectionResult.STALE
        }
        val historical = history.lookup(timestampNanos)
        if (historical == null) {
            onEvent("pose correction rejected: timestamp outside pose history")
            return CorrectionResult.NO_HISTORY
        }
        val current = follower.pose
        val relative = relativePose(historical, current)
        val corrected = composePose(measured, relative)

        val jumpInches = hypot(corrected.x - current.x, corrected.y - current.y)
        val headingDelta = shortestHeadingDelta(current.heading, corrected.heading)
        if (jumpInches > maxJumpInches || abs(headingDelta) > maxJumpRadians) {
            onEvent(
                "pose correction rejected: jump %.1f in / %.1f deg exceeds gate".format(
                    Locale.US, jumpInches, Math.toDegrees(abs(headingDelta)),
                ),
            )
            return CorrectionResult.REJECTED_JUMP
        }

        val b = blend.coerceIn(0.0, 1.0)
        follower.setPose(
            Pose(
                current.x + (corrected.x - current.x) * b,
                current.y + (corrected.y - current.y) * b,
                MathFunctions.normalizeAngle(current.heading + headingDelta * b),
            ),
        )
        onEvent(
            "pose correction applied: %.1f in / %.1f deg at blend %.2f".format(
                Locale.US, jumpInches, Math.toDegrees(abs(headingDelta)), b,
            ),
        )
        return CorrectionResult.APPLIED
    }
}
