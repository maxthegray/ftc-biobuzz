package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase

/**
 * Flight-log hook for the ball assists. No hardware, no periodic work — it
 * exists because per-tick WPILOG channels come only from
 * [SubsystemBase.logState], and the two controllers are plain classes the
 * op-mode owns.
 *
 * Logs the numbers needed to explain an assist after the fact: the target the
 * controllers actually acted on, the powers they asked for, and the gains in
 * force at the time. The `Limelight` channels already record what the camera
 * saw, but that is its *primary* target — [BallAimController.selectTarget]
 * picks the largest blob, so with two balls in frame the two disagree and only
 * this one explains the output.
 *
 * Channels read NaN / zero whenever no assist is engaged, which makes the held
 * windows obvious in the log.
 */
class BallAssistSubsystem(
    private val aim: BallAimController,
    private val range: BallRangeController,
) : SubsystemBase("BallAssist") {

    override fun logState(log: StateLog) {
        log.put("tx", aim.lastTxDegrees)
        log.put("ty", range.lastTyDegrees)
        log.put("turnPower", aim.lastOutput)
        log.put("forwardPower", range.lastOutput)
        // The gains are live-tuned mid-session, so a log without them can't say
        // which kP produced the behaviour it recorded.
        log.put("aimKp", BallAimConfig.kP)
        log.put("approachKp", BallApproachConfig.kP)
        log.put("targetTy", BallApproachConfig.targetTyDegrees)
    }
}
