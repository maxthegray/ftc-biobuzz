package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.vision.TagSighting

/**
 * The Limelight 3A on pipeline [PIPELINE]: whether its latest result is fresh,
 * and every AprilTag in it. Read-only; which tags are the goal is
 * [org.firstinspires.ftc.teamcode.vision.GoalAim]'s job.
 */
class LimelightSubsystem : SubsystemBase("Limelight") {

    private lateinit var limelight: Limelight3A

    var connected = false
        private set

    /** The latest result is valid and less than [MAX_AGE_MS] old. */
    var fresh = false
        private set

    /** Every AprilTag in the latest fresh result; empty when not [fresh]. */
    var tags: List<TagSighting> = emptyList()
        private set

    private var targetTxDeg = 0.0
    private var targetTyDeg = 0.0

    override fun init(hardwareMap: HardwareMap) {
        limelight = hardwareMap.get(Limelight3A::class.java, RobotConfig.Vision.LIMELIGHT)
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(PIPELINE)
        limelight.start()
    }

    override fun periodic() {
        connected = limelight.isConnected
        val result = limelight.latestResult?.takeIf { connected && it.isValid && it.staleness < MAX_AGE_MS }
        fresh = result != null
        tags = result?.fiducialResults?.map { TagSighting(it.fiducialId, it.targetXDegrees, it.targetYDegrees) } ?: emptyList()
        targetTxDeg = result?.tx ?: 0.0
        targetTyDeg = result?.ty ?: 0.0
    }

    override fun health(): String = when {
        !connected -> "disconnected"
        !fresh -> "no fresh result"
        else -> "${tags.size} tag(s)"
    }

    override fun logState(log: StateLog) {
        log.put("connected", connected)
        log.put("target/visible", fresh)
        log.put("target/txDegrees", targetTxDeg)
        log.put("target/tyDegrees", targetTyDeg)
        log.put("tags/id", DoubleArray(tags.size) { tags[it].id.toDouble() })
        log.put("tags/txDegrees", DoubleArray(tags.size) { tags[it].txDeg })
        log.put("tags/tyDegrees", DoubleArray(tags.size) { tags[it].tyDeg })
    }

    override fun stop() {
        if (::limelight.isInitialized) limelight.stop()
    }

    private companion object {
        const val PIPELINE = 0
        const val MAX_AGE_MS = 100L
    }
}
