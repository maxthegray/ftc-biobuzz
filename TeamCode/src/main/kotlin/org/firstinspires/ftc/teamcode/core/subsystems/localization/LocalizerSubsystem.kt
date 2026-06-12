package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.follower.Follower
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.estimation.CorrectionResult
import org.firstinspires.ftc.teamcode.core.estimation.PoseEstimator
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.Vector2d
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.pathing.toCore
import org.firstinspires.ftc.teamcode.core.pathing.toPedro
import org.firstinspires.ftc.teamcode.core.runtime.DeviceReaders
import org.firstinspires.ftc.teamcode.core.runtime.PersistedPose
import org.firstinspires.ftc.teamcode.core.runtime.PoseProvider
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig
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
 *
 * A runtime **watchdog** ([periodic]) catches the localizer dying mid-match
 * — the failure everything downstream silently trusts not to happen. It trips
 * on a non-finite pose, on a pose frozen bit-identical for
 * [LocalizerConfig.frozenPoseTicks] ticks while a path is being followed, or
 * on a non-READY Pinpoint device status (checked ~1 Hz, only when the raw
 * Pinpoint is in the hardware map). A trip latches [fault], surfaces in
 * [health] and the flight log, and fires [onFault] once — wire the policy
 * there (teleop: break the path, driver keeps stick control; auton: cancel
 * the routine and stop, because driving blind is worse than parking).
 */
class LocalizerSubsystem(
    private val follower: Follower,
    private val clock: Clock = Clock.SYSTEM,
    private val onEvent: (String) -> Unit = {},
    private val isFollowing: () -> Boolean = { false },
    private val onFault: () -> Unit = {},
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

    /** Latched watchdog fault, or null while healthy. Cleared only by re-init. */
    var fault: String? = null
        private set

    private var rawPinpoint: GoBildaPinpointDriver? = null
    private var lastStatusNs = Long.MIN_VALUE
    private var frozenTicks = 0
    private var lastPose = Pose2d.ZERO
    private var hasLastPose = false

    override fun init(hardwareMap: HardwareMap) {
        // The follower owns the localizer; the raw Pinpoint (when present —
        // not with the SRSHub variant or in host tests) is only for the
        // watchdog's device-status check.
        rawPinpoint = try {
            DeviceReaders.maybe(
                hardwareMap,
                RobotConfig.Localization.PINPOINT,
                GoBildaPinpointDriver::class.java,
            )
        } catch (_: Throwable) {
            null
        }
    }

    override fun periodic() {
        if (fault != null || !LocalizerConfig.watchdogEnabled) return
        val p = pose
        if (!p.x.isFinite() || !p.y.isFinite() || !p.heading.isFinite()) {
            trip("non-finite pose $p")
            return
        }
        // A live Pinpoint jitters at the float level every read; a pose that
        // stays bit-identical while the follower is commanding motion means
        // the sensor stopped talking. Gated on following so a parked robot
        // can't false-positive.
        if (isFollowing() && hasLastPose &&
            p.x == lastPose.x && p.y == lastPose.y && p.heading == lastPose.heading
        ) {
            frozenTicks++
            if (frozenTicks >= LocalizerConfig.safeFrozenPoseTicks) {
                trip("pose frozen for $frozenTicks ticks while following")
                return
            }
        } else {
            frozenTicks = 0
        }
        lastPose = p
        hasLastPose = true

        val pinpoint = rawPinpoint ?: return
        val now = clock.nanos()
        if (lastStatusNs != Long.MIN_VALUE && now - lastStatusNs < STATUS_INTERVAL_NS) return
        lastStatusNs = now
        val status = try {
            pinpoint.deviceStatus
        } catch (_: Throwable) {
            null // a flaky status read alone shouldn't kill localization
        }
        if (status != null && status != GoBildaPinpointDriver.DeviceStatus.READY) {
            trip("Pinpoint status $status")
        }
    }

    private fun trip(reason: String) {
        fault = reason
        onEvent("LOCALIZER FAULT: $reason")
        try {
            onFault()
        } catch (_: Throwable) {
            // The policy callback is best-effort; the latch + event already
            // carry the diagnosis, and periodic() must keep the loop alive.
        }
    }

    override fun health(): String = fault?.let { "FAULT: $it" } ?: "ok"

    override fun logState(log: StateLog) {
        log.put("faulted", fault != null)
        fault?.let { log.put("fault", it) }
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

    private companion object {
        const val STATUS_INTERVAL_NS = 1_000_000_000L
    }
}
