package org.firstinspires.ftc.teamcode.core.hardware

import org.junit.Test

class SRSHubSubsystemTest {

    /**
     * Pedro's PoseTracker calls the localizer's resetIMU() while the
     * Follower is being *constructed* — in configure(), before any
     * subsystem's init() has resolved the hub. The handle must queue the
     * command instead of touching the uninitialised hub.
     */
    @Test
    fun pinpointCommandsBeforeInitAreQueuedNotCrashing() {
        val subsystem = SRSHubSubsystem()
        val pinpoint = subsystem.pinpoint(
            bus = 1,
            xPodOffsetMm = 84f,
            yPodOffsetMm = -168f,
            ticksPerMm = 13.26f,
            xDir = SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD,
            yDir = SRSHub.GoBildaPinpoint.EncoderDirection.REVERSED,
        )

        pinpoint.resetImu()
        pinpoint.setPose(0f, 0f, 0f)

        // periodic() before init must stay a no-op too.
        subsystem.periodic()
    }
}
