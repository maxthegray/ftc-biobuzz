package org.firstinspires.ftc.teamcode.core.sim

import org.firstinspires.ftc.teamcode.core.io.ServoIO

/** [ServoIO] that records what was written. */
class FakeServoIO : ServoIO {
    override var lastPosition: Double = Double.NaN
        private set

    var writes = 0
        private set

    override fun setPosition(position: Double) {
        lastPosition = position
        writes++
    }
}
