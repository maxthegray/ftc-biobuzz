package org.firstinspires.ftc.teamcode.vision

import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.subsystems.LimelightFiducial
import org.firstinspires.ftc.teamcode.subsystems.LimelightPose
import org.firstinspires.ftc.teamcode.subsystems.LimelightReading
import org.firstinspires.ftc.teamcode.subsystems.LimelightSource
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.Cell
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.CellLocation

/**
 * Synthetic Limelight fiducials for [HiveConfig]'s default mount: a level
 * camera at the robot's pose point on the tiles with the turret at 0, so camera
 * (right, down, out) = robot (−y, −z, x). Every tag squarely faces the lens.
 */
internal object HiveTestFrames {

    /** Goal height of a raised CELL; its tags sit 7.19 in lower, well above the 43.95 in threshold. */
    const val RAISED_GOAL_HEIGHT_IN = 60.0

    /** Goal height of a lowered CELL; its tags sit well below the threshold. */
    const val LOWERED_GOAL_HEIGHT_IN = 36.0

    fun cell(alliance: Alliance, location: CellLocation) = Cell(alliance, location)

    /** Tags of [cell] whose solved poses all imply a goal at [goalRobot]; [perturbIn] moves single tags (camera frame). */
    fun cluster(cell: Cell, goalRobot: Vec3, perturbIn: Map<Int, Vec3> = emptyMap()): List<LimelightFiducial> {
        val goalCam = Vec3(-goalRobot.y, -goalRobot.z, goalRobot.x)
        return BiobuzzAprilTags.tagsOf(cell).map { tag ->
            val centre = goalCam +
                Vec3(tag.offsetFromClusterCenterInches, BiobuzzAprilTags.TAG_ROW_Y_INCHES, BiobuzzAprilTags.TAG_ROW_Z_INCHES) +
                (perturbIn[tag.id] ?: Vec3(0.0, 0.0, 0.0))
            fiducial(tag.id, centre)
        }
    }

    fun raised(alliance: Alliance, location: CellLocation, forwardIn: Double = 80.0, leftIn: Double = 0.0) =
        cluster(cell(alliance, location), Vec3(forwardIn, leftIn, RAISED_GOAL_HEIGHT_IN))

    fun lowered(alliance: Alliance, location: CellLocation, forwardIn: Double = 80.0, leftIn: Double = 0.0) =
        cluster(cell(alliance, location), Vec3(forwardIn, leftIn, LOWERED_GOAL_HEIGHT_IN))

    fun fiducial(id: Int, centreCamInches: Vec3, family: String = "36H11C") = LimelightFiducial(
        id = id,
        family = family,
        txDegrees = 0.0,
        tyDegrees = 0.0,
        txNoCrosshairDegrees = 0.0,
        tyNoCrosshairDegrees = 0.0,
        areaPercent = 1.0,
        targetPoseCameraSpace = LimelightPose(
            centreCamInches.x * 0.0254,
            centreCamInches.y * 0.0254,
            centreCamInches.z * 0.0254,
            0.0,
            0.0,
            0.0,
        ),
        cameraPoseTargetSpace = null,
        cornerCount = 4,
    )
}

/** A [HiveTracker] fed through a real [LimelightSubsystem] on a fake source and clock. */
internal class HiveRig(priors: Map<Alliance, HiveState?> = HiveTracker.MATCH_SETUP) {
    val clock = FakeClock()
    val source = FakeLimelightSource()
    val limelight = LimelightSubsystem(source = source, clock = clock)
    var turretAngle: Double? = 0.0
    val requestedAngleTimes = ArrayList<Long>()

    /** Returned both as the pose at capture and as the current pose; null means no localizer. */
    var pose: Pose? = null
    val events = ArrayList<String>()
    val tracker = HiveTracker(
        limelight,
        { t -> requestedAngleTimes += t; turretAngle },
        priors,
        poseAt = { pose },
        currentPose = { pose },
        eventSink = events::add,
        clock = clock,
    )
    private var frameTs = 100.0

    init {
        limelight.init(HardwareMap(null, null))
        source.isConnected = true
    }

    /** One new Limelight frame, 20 ms after the previous tick. Capture time = now − age − latencies. */
    fun frame(
        fiducials: List<LimelightFiducial>,
        ageMs: Long = 0L,
        captureMs: Double = 0.0,
        targetingMs: Double = 0.0,
        pipeline: Int = 0,
    ) {
        clock.advanceMs(20.0)
        frameTs += 20.0
        source.reading = LimelightReading(
            receiptTimestampMs = frameTs.toLong(),
            ageMs = ageMs,
            valid = fiducials.isNotEmpty(),
            pipelineIndex = pipeline,
            pipelineType = "pipe_fiducial",
            captureLatencyMs = captureMs,
            targetingLatencyMs = targetingMs,
            limelightTimestampMs = frameTs,
            fiducials = fiducials,
        )
        limelight.periodic()
        tracker.periodic()
    }

    /** A tick with no new frame. */
    fun idle(ms: Double) {
        clock.advanceMs(ms)
        limelight.periodic()
        tracker.periodic()
    }
}

internal class FakeLimelightSource : LimelightSource {
    override var isRunning = false
    override var isConnected = false
    var reading = LimelightReading()

    override fun setPollRateHz(rateHz: Int) {}
    override fun pipelineSwitch(index: Int): Boolean = true
    override fun start() {
        isRunning = true
    }
    override fun latestReading(): LimelightReading = reading
    override fun stop() {
        isRunning = false
    }
}
