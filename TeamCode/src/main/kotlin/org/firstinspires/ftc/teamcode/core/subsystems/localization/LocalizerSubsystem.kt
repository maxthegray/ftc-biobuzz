package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.estimation.CorrectionResult
import org.firstinspires.ftc.teamcode.core.estimation.PoseEstimator
import org.firstinspires.ftc.teamcode.core.estimation.isFinite
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.DeviceReaders
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * Read-only façade over the [Follower]'s localizer, plus the
 * external-correction seam (vision, wall snaps) via [PoseEstimator].
 *
 * Pedro's Follower owns the real localizer (the Pinpoint, configured in
 * [org.firstinspires.ftc.teamcode.pedro.Constants]).
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
 * **Fresh localization every run.** [init] writes [startingPose] (zero unless
 * the op-mode configures a field start pose) to the Pinpoint, so nothing the
 * device still holds from a previous op-mode survives. A read that disagrees
 * with the start pose before one has confirmed it (the device can report a
 * pre-reset sample) re-writes it; [ready] stays false until a read confirms it,
 * and the start pose is abandoned as a fault after five seconds. The pose is
 * only written, never recalibrated: the IMU keeps its power-up calibration.
 * There is no pose carryover between op-modes.
 *
 * A runtime **watchdog** ([periodic]) catches the localizer dying mid-match
 * — the failure everything downstream silently trusts not to happen. It trips
 * on a non-finite pose, on a pose frozen bit-identical for
 * [LocalizerConfig.frozenPoseTicks] ticks while a path is being followed, or
 * on a non-READY Pinpoint device status (checked ~1 Hz during the run, only
 * when the raw Pinpoint is in the hardware map). [initPeriodic] refreshes
 * odometry without actuator writes and allows up to five seconds for initial
 * NOT_READY/CALIBRATING status; [ready] gates autonomous startup.
 * A trip latches [fault], surfaces in
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
    /** Pose written at every init. Autonomous passes its field start pose. */
    val startingPose: Pose = Pose.zero(),
) : SubsystemBase("Localizer") {

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
    private var pinpointReady = false
    private var initStartedNs = 0L
    private var lastStatus: DeviceStatus? = null
    private var startPoseConfirmed = false

    private var lastStatusNs = Long.MIN_VALUE
    private var frozenTicks = 0
    private var lastPose = Pose.zero()
    private var hasLastPose = false

    /** The start pose is confirmed by a read and a real status sample reports READY. */
    val ready: Boolean get() = fault == null && startPoseConfirmed && (rawPinpoint == null || pinpointReady)

    override fun init(hardwareMap: HardwareMap) {
        initStartedNs = clock.nanos()
        // The follower owns the localizer; the raw Pinpoint is resolved
        // separately for the watchdog's device-status check. It may be absent
        // in host tests.
        rawPinpoint = try {
            DeviceReaders.maybe(
                hardwareMap,
                RobotConfig.Localization.PINPOINT,
                GoBildaPinpointDriver::class.java,
            )
        } catch (_: Throwable) {
            null
        }
        startPoseConfirmed = false
        follower.setPose(startingPose)
    }

    override fun initPeriodic() {
        if (fault != null) return
        try {
            // Reads odometry only; Follower.update would also drive motors.
            follower.localizer.update()
        } catch (t: Exception) {
            trip("odometry init read failed: ${t.message}")
            return
        }
        checkPinpointStatus(initializing = true)
        if (fault == null) confirmStartPose()
        if (ready) checkPose()
    }

    override fun periodic() {
        if (fault != null) return
        // Started before a read confirmed the start pose (START pressed right after INIT).
        if (!startPoseConfirmed) confirmStartPose()
        if (fault != null || !LocalizerConfig.watchdogEnabled) return
        checkPinpointStatus(initializing = false)
        if (fault == null) checkPose()
    }

    private fun confirmStartPose() {
        val p = pose
        if (!p.isFinite()) {
            // A broken sensor, not a stale sample: never paper over it with a write.
            trip("non-finite pose $p")
            return
        }
        val offBy = p.distance(startingPose)
        if (offBy <= START_POSE_TOLERANCE_INCHES &&
            kotlin.math.abs(shortestAngleDelta(p.heading(), startingPose.heading())) <= START_POSE_TOLERANCE_RADIANS
        ) {
            startPoseConfirmed = true
            return
        }
        if (clock.nanos() - initStartedNs >= STARTUP_TIMEOUT_NS) {
            trip("start pose not confirmed: Pinpoint still reports $p")
            return
        }
        // A sample from before the reset: write the start pose again.
        follower.setPose(startingPose)
    }

    private fun checkPose() {
        val p = pose
        if (!p.isFinite()) {
            trip("non-finite pose $p")
            return
        }
        // A live Pinpoint jitters at the float level every read; a pose that
        // stays bit-identical while the follower is commanding motion means
        // the sensor stopped talking. Gated on following so a parked robot
        // can't false-positive.
        if (isFollowing() && hasLastPose &&
            p.x() == lastPose.x() && p.y() == lastPose.y() && p.heading() == lastPose.heading()
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
    }

    private fun checkPinpointStatus(initializing: Boolean) {
        val pinpoint = rawPinpoint ?: return
        val now = clock.nanos()
        if (!initializing && pinpointReady && lastStatusNs != Long.MIN_VALUE &&
            now - lastStatusNs < STATUS_INTERVAL_NS
        ) return
        lastStatusNs = now
        val status = try {
            pinpoint.deviceStatus
        } catch (t: Exception) {
            trip("Pinpoint status read failed: ${t.message}")
            return
        }
        lastStatus = status
        when {
            status == DeviceStatus.READY -> pinpointReady = true
            initializing && !pinpointReady &&
                (status == DeviceStatus.NOT_READY || status == DeviceStatus.CALIBRATING) -> {
                if (now - initStartedNs >= STARTUP_TIMEOUT_NS) {
                    trip("Pinpoint startup timed out: $status")
                }
            }
            else -> trip("Pinpoint status $status")
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

    override fun health(): String = fault?.let { "FAULT: $it" } ?: when {
        ready -> "ok"
        !startPoseConfirmed -> "waiting for start pose ${startingPose} (status ${lastStatus ?: "unread"})"
        else -> "waiting for Pinpoint READY (status ${lastStatus ?: "unread"})"
    }

    override fun logState(log: StateLog) {
        log.put("faulted", fault != null)
        log.put("startPoseConfirmed", startPoseConfirmed)
        fault?.let { log.put("fault", it) }
    }

    override fun writeHardware() {
        // Not a hardware write — this runs here (after the drive subsystem's
        // Follower.update()) so the sample timestamp matches when the pose
        // was measured. See the class doc.
        estimator.sample(clock.nanos(), pose)
    }

    /** Field pose: inches, radians. */
    val pose: Pose get() = follower.pose()

    /** Field-frame velocity: inches/second, radians/second. */
    val velocity: Velocity get() = follower.velocity()

    /** Hard-set the field pose (relocalization, e.g. a wall snap). */
    fun setPose(p: Pose) {
        follower.setPose(p)
    }

    /**
     * Apply a delayed field-pose measurement while preserving motion since
     * [timestampNanos] — see [PoseEstimator.applyCorrection] for gating,
     * blending, axis weights, and the during-follow policy.
     */
    fun applyCorrection(
        measured: Pose,
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
        const val STARTUP_TIMEOUT_NS = 5_000_000_000L

        /** How far (inches) a read may be from the start pose and still confirm it. */
        const val START_POSE_TOLERANCE_INCHES = 0.5

        /** How far (radians, 1°) a read's heading may be from the start pose's and still confirm it. */
        val START_POSE_TOLERANCE_RADIANS = Math.toRadians(1.0)
    }
}
