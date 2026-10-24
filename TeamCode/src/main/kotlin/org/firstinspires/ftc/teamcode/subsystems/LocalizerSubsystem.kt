package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Clock
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.Events
import org.firstinspires.ftc.teamcode.core.logging.StateLog

/**
 * The robot's pose, read from the Pinpoint through Pedro's [Follower], plus
 * the checks that make it trustworthy in a match.
 *
 * **Fresh start pose every run.** [init] writes [startingPose] (an auto passes
 * its field start pose) to the Pinpoint. The device can still report a sample
 * from before the write, so [ready] stays false until a read confirms the pose
 * (it is written again during init, and gives up after 5 s). Nothing carries
 * over between op-modes, and the IMU is never recalibrated.
 *
 * **Watchdog**, two kinds of trip, each latched and reported once:
 *  - [fault]: the pose can't be driven on (non-finite, frozen while a path is
 *    followed, unreadable). Calls [onFault] and ends the checks. Teleop falls
 *    back to robot-centric sticks; an auto cancels its routine and parks.
 *  - [statusFault]: the Pinpoint reported a problem (bad status, start pose
 *    never confirmed) but the pose is still live, so the pose checks go on.
 *    Calls [onStatusFault]. Teleop treats it like [onFault]; autos pass `{}`
 *    and keep driving.
 *
 * **Pose history** for vision latency compensation ([poseAt]) is sampled in
 * [writeHardware], right after the drive's `Follower.update()`, so each sample
 * is stamped when it was measured. That is why this must be registered after
 * the drive ([registerAfter]).
 */
class LocalizerSubsystem(
    private val follower: Follower,
    private val clock: Clock = Clock.SYSTEM,
    private val isFollowing: () -> Boolean = { false },
    /** The pose can't be driven on. */
    private val onFault: () -> Unit,
    /** The Pinpoint reported a problem but the pose is still live. */
    private val onStatusFault: () -> Unit,
    /** Pose written at every init. Autonomous passes its field start pose. */
    val startingPose: Pose = Pose.zero(),
) : SubsystemBase("Localizer") {

    private val history = PoseHistory()

    override val registerAfter: Class<out SubsystemBase>
        get() = MecanumDriveSubsystem::class.java

    /** Latched pose fault, or null while healthy. Cleared only by re-init. */
    var fault: String? = null
        private set

    /** Latched Pinpoint problem with the pose still live, or null. Cleared only by re-init. */
    var statusFault: String? = null
        private set

    private var rawPinpoint: GoBildaPinpointDriver? = null
    private var pinpointReady = false
    private var initStartedNs = 0L
    private var lastStatus: DeviceStatus? = null
    private var startPoseConfirmed = false

    private var badReadsInARow = 0
    private var frozenTicks = 0
    private var lastPose = Pose.zero()
    private var hasLastPose = false

    /** The start pose is confirmed by a read and a real status sample reports READY. */
    val ready: Boolean get() =
        fault == null && statusFault == null && startPoseConfirmed && (rawPinpoint == null || pinpointReady)

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
        badReadsInARow = 0
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
        if (statusFault == null) checkPinpointStatus(initializing = true)
        if (statusFault == null) confirmStartPose()
        if (fault == null && startPoseConfirmed) checkPose()
    }

    override fun periodic() {
        if (fault != null) return
        // Started before a read confirmed the start pose (START pressed right after INIT).
        if (!startPoseConfirmed && statusFault == null) confirmStartPose()
        if (fault != null) return
        if (statusFault == null) checkPinpointStatus(initializing = false)
        checkPose()
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
        // Once confirmed, a miss is the robot being moved during init, not a stale sample.
        if (!startPoseConfirmed && clock.nanos() - initStartedNs >= STARTUP_TIMEOUT_NS) {
            tripStatus("start pose not confirmed: Pinpoint still reports $p")
            return
        }
        // A sample from before the reset, or the robot nudged into place: write the start pose again.
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

    /** The driver caches the status from the last bulk read, so this costs no I2C and runs every tick. */
    private fun checkPinpointStatus(initializing: Boolean) {
        val pinpoint = rawPinpoint ?: return
        val now = clock.nanos()
        val status = try {
            pinpoint.deviceStatus
        } catch (t: Exception) {
            tripStatus("Pinpoint status read failed: ${t.message}")
            return
        }
        lastStatus = status
        // The SDK driver flags one corrupt sample this way and keeps the previous
        // value; the next read clears it. Only a run of them is a dead sensor.
        if (status == DeviceStatus.FAULT_BAD_READ) {
            badReadsInARow++
            if (badReadsInARow >= BAD_READS_TO_FAULT) tripStatus("Pinpoint status $status for $badReadsInARow reads in a row")
            return
        }
        badReadsInARow = 0
        when {
            status == DeviceStatus.READY -> pinpointReady = true
            initializing && !pinpointReady &&
                (status == DeviceStatus.NOT_READY || status == DeviceStatus.CALIBRATING) -> {
                if (now - initStartedNs >= STARTUP_TIMEOUT_NS) {
                    tripStatus("Pinpoint startup timed out: $status")
                }
            }
            else -> tripStatus("Pinpoint status $status")
        }
    }

    private fun trip(reason: String) {
        fault = reason
        Events.record("LOCALIZER FAULT: $reason")
        runPolicy(onFault)
    }

    private fun tripStatus(reason: String) {
        statusFault = reason
        Events.record("LOCALIZER FAULT (pose still live): $reason")
        runPolicy(onStatusFault)
    }

    private fun runPolicy(policy: () -> Unit) {
        try {
            policy()
        } catch (_: Throwable) {
            // The policy callback is best-effort; the latch + event already
            // carry the diagnosis, and periodic() must keep the loop alive.
        }
    }

    override fun health(): String = fault?.let { "FAULT: $it" } ?: statusFault?.let { "FAULT (pose still live): $it" } ?: when {
        ready -> "ok"
        !startPoseConfirmed -> "waiting for start pose ${startingPose} (status ${lastStatus ?: "unread"})"
        else -> "waiting for Pinpoint READY (status ${lastStatus ?: "unread"})"
    }

    override fun logState(log: StateLog) {
        log.put("faulted", fault != null || statusFault != null)
        log.put("startPoseConfirmed", startPoseConfirmed)
        (fault ?: statusFault)?.let { log.put("fault", it) }
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
        const val STARTUP_TIMEOUT_NS = 5_000_000_000L

        /** Consecutive FAULT_BAD_READ samples (one per tick) before the Pinpoint is called dead. */
        const val BAD_READS_TO_FAULT = 5

        /** How far (inches) a read may be from the start pose and still confirm it. */
        const val START_POSE_TOLERANCE_INCHES = 0.5

        /** How far (radians, 1°) a read's heading may be from the start pose's and still confirm it. */
        val START_POSE_TOLERANCE_RADIANS = Math.toRadians(1.0)
    }
}

/** Localizer tuning, live-editable in Panels; copy values you want to keep into this file. */
@Configurable
object LocalizerConfig {
    /**
     * Ticks of a bit-identical pose while following before the watchdog calls
     * the localizer dead. A live Pinpoint jitters every read. 25 ≈ 0.5 s at 50 Hz.
     */
    @JvmField var frozenPoseTicks = 25
}
