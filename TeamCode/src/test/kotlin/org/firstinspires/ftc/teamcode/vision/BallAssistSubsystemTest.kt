package org.firstinspires.ftc.teamcode.vision

import kotlin.test.assertEquals
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.junit.Test

class BallAssistSubsystemTest {

    @Test
    fun logsTheTargetItActedOnTheOutputsAndTheGains() {
        val aim = BallAimController()
        val range = BallRangeController()
        val savedAimKp = BallAimConfig.kP
        val savedApproachKp = BallApproachConfig.kP
        val savedTargetTy = BallApproachConfig.targetTyDegrees
        try {
            BallAimConfig.kP = 0.02
            BallApproachConfig.kP = 0.05
            BallApproachConfig.targetTyDegrees = -10.0
            BallApproachConfig.alignGateDegrees = 25.0
            aim.update(DT, txDegrees = 8.0)
            range.update(DT, tyDegrees = -4.0, txDegrees = 8.0)
            val log = RecordingStateLog()

            BallAssistSubsystem(aim, range).logState(log)

            assertEquals(8.0, log.channels["tx"])
            assertEquals(-4.0, log.channels["ty"])
            assertEquals(aim.lastOutput, log.channels["turnPower"])
            assertEquals(range.lastOutput, log.channels["forwardPower"])
            assertEquals(0.02, log.channels["aimKp"])
            assertEquals(0.05, log.channels["approachKp"])
            assertEquals(-10.0, log.channels["targetTy"])
        } finally {
            BallAimConfig.kP = savedAimKp
            BallApproachConfig.kP = savedApproachKp
            BallApproachConfig.targetTyDegrees = savedTargetTy
        }
    }

    private class RecordingStateLog : StateLog {
        val channels = mutableMapOf<String, Any>()
        override fun put(channel: String, value: Double) { channels[channel] = value }
        override fun put(channel: String, value: Long) { channels[channel] = value }
        override fun put(channel: String, value: Boolean) { channels[channel] = value }
        override fun put(channel: String, value: String) { channels[channel] = value }
    }

    private companion object {
        const val DT = 0.02
    }
}
