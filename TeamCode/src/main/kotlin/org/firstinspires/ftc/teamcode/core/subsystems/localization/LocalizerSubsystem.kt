package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.Clock
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase

/**
 * The robot's pose, read from the Pinpoint through Pedro's [Follower], plus
 * the checks that make it trustworthy in a match.
 *
 * **Fresh start pose every run.** [init] writes [startingPose] (an auto passes
 * its field start pose) to the Pinpoint. The device can still report a sample
 * from before the write, so [ready] stays false until a read confirms the pose
 * (it is written again until then, and gives up as a fault after 5 s). Nothing
 * carries over between op-modes, and the IMU is never recalibrated.
 *
 * **Watchdog.** [periodic] trips on a non-finite pose, a pose frozen while a
 * path is being followed, or a bad Pinpoint status. A trip latches [fault]
 * and calls [onFault] once: teleop falls back to robot-centric sticks, an auto
 * cancels its routine and stops.
 *
 * **Pose history** for vision latency compensation ([poseAt]) is sampled in
 * [writeHardware], right after the drive's `Follower.update()`, so each sample
 * is stamped when it was measured. That is why this must be registered after
 * the drive ([registerAfter]).
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

    private val history = PoseHistory()

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
            hardwareMap.tryGet(GoBildaPinpointDriver::class.java, RobotConfig.Localization.PINPOINT)
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
            if (frozenTicks >= LocalizerConfig.frozenPoseTicks) {
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
        // Not a hardware write: this is the first point after Follower.update().
        history.add(clock.nanos(), pose)
    }

    /** The pose at a past [Clock] time, interpolated; null outside the last 512 samples (a few seconds). */
    fun poseAt(timestampNanos: Long): Pose? = history.lookup(timestampNanos)

    /** Field pose: inches, radians. */
    val pose: Pose get() = follower.pose()

    /** Field-frame velocity: inches/second, radians/second. */
    val velocity: Velocity get() = follower.velocity()

    /** Hard-set the field pose (relocalization, e.g. a wall snap). */
    fun setPose(p: Pose) {
        follower.setPose(p)
    }

    private companion object {
        const val STATUS_INTERVAL_NS = 1_000_000_000L
        const val STARTUP_TIMEOUT_NS = 5_000_000_000L

        /** How far (inches) a read may be from the start pose and still confirm it. */
        const val START_POSE_TOLERANCE_INCHES = 0.5

        /** How far (radians, 1°) a read's heading may be from the start pose's and still confirm it. */
        val START_POSE_TOLERANCE_RADIANS = Math.toRadians(1.0)
    }
}

/** Localizer tuning, live-editable in Panels; copy values you want to keep into this file. */
@Configurable
object LocalizerConfig {
    /** Off switch in case the watchdog ever false-trips. */
    @JvmField var watchdogEnabled = true

    /**
     * Ticks of a bit-identical pose while following before the watchdog calls
     * the localizer dead. A live Pinpoint jitters every read. 25 ≈ 0.5 s at 50 Hz.
     */
    @JvmField var frozenPoseTicks = 25
}
