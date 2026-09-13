package org.firstinspires.ftc.teamcode.core.subsystems.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class LimelightSubsystemTest {

    private val clock = FakeClock()
    private val source = FakeLimelightSource()
    private val subsystem = LimelightSubsystem(source = source, clock = clock)

    @Test
    fun initConfiguresPipelineAndStartsPolling() {
        subsystem.init(HardwareMap(null, null))

        assertEquals(100, source.configuredPollRateHz)
        assertEquals(0, source.configuredPipelineIndex)
        assertTrue(source.isRunning)
        assertTrue(subsystem.pipelineSwitchAccepted)
    }

    @Test
    fun freshColorResultPublishesPrimaryAndAllTargets() {
        startWith(
            LimelightReading(
                receiptTimestampMs = 10L,
                ageMs = 15L,
                valid = true,
                pipelineIndex = 0,
                pipelineType = "color",
                txDegrees = -4.5,
                tyDegrees = 7.0,
                areaPercent = 3.25,
                colorTargets = listOf(
                    LimelightColorTarget(-4.5, 7.0, 3.25),
                    LimelightColorTarget(12.0, 5.0, 1.5),
                ),
            ),
        )

        assertTrue(subsystem.resultFresh)
        assertTrue(subsystem.targetVisible)
        assertEquals(2, subsystem.targetCount)
        assertEquals(-4.5, subsystem.primaryTarget!!.txDegrees, 0.0)
        assertEquals("tracking 2 target(s)", subsystem.health())
    }

    @Test
    fun staleResultCannotRemainVisible() {
        startWith(
            LimelightReading(
                receiptTimestampMs = 10L,
                ageMs = 101L,
                valid = true,
                pipelineIndex = 0,
                colorTargets = listOf(LimelightColorTarget(2.0, 3.0, 4.0)),
            ),
        )

        assertFalse(subsystem.resultFresh)
        assertFalse(subsystem.targetVisible)
        assertNull(subsystem.primaryTarget)
        assertEquals(0, subsystem.targetCount)
        assertEquals(1L, subsystem.staleTickCount)
    }

    @Test
    fun pipelineMismatchCannotPublishTarget() {
        startWith(
            LimelightReading(
                receiptTimestampMs = 10L,
                ageMs = 5L,
                valid = true,
                pipelineIndex = 1,
            ),
        )

        assertFalse(subsystem.pipelineMatches)
        assertFalse(subsystem.targetVisible)
        assertEquals("pipeline 1 active; expected 0", subsystem.health())
    }

    @Test
    fun resultRateCountsOnlyNewFrames() {
        startWith(LimelightReading(receiptTimestampMs = 1L, ageMs = 0L, pipelineIndex = 0))
        repeat(4) {
            clock.advanceMs(100.0)
            subsystem.periodic()
        }
        repeat(5) { index ->
            source.reading = source.reading.copy(receiptTimestampMs = 2L + index)
            clock.advanceMs(120.0)
            subsystem.periodic()
        }

        assertEquals(6L, subsystem.receivedFrameCount)
        assertEquals(6.0, subsystem.resultRateHz, 0.001)
    }

    @Test
    fun freshAprilTagResultPublishesFiducialsWithTiming() {
        startWith(
            LimelightReading(
                receiptTimestampMs = 10L,
                ageMs = 20L,
                valid = true,
                pipelineIndex = 0,
                pipelineType = "pipe_fiducial",
                captureLatencyMs = 11.0,
                targetingLatencyMs = 7.0,
                limelightTimestampMs = 5000.0,
                fiducials = listOf(fiducial(34), fiducial(35)),
            ),
        )

        assertEquals(listOf(34, 35), subsystem.fiducials.map { it.id })
        assertEquals(38.0, subsystem.estimatedCaptureAgeMs, 1e-9)
        assertEquals(5000.0, subsystem.limelightTimestampMs, 0.0)
        assertEquals("tracking 2 tag(s)", subsystem.health())
    }

    @Test
    fun fiducialsClearWhenStaleOrWrongPipeline() {
        startWith(
            LimelightReading(
                receiptTimestampMs = 10L, ageMs = 5L, valid = true, pipelineIndex = 0,
                limelightTimestampMs = 1.0, fiducials = listOf(fiducial(40)),
            ),
        )
        assertEquals(1, subsystem.fiducials.size)

        source.reading = source.reading.copy(ageMs = 250L)
        subsystem.periodic()
        assertTrue(subsystem.fiducials.isEmpty())

        source.reading = source.reading.copy(ageMs = 5L, pipelineIndex = 3)
        subsystem.periodic()
        assertTrue(subsystem.fiducials.isEmpty())
    }

    @Test
    fun deviceTimestampIdentifiesFramesWhenPollsRepeatAResult() {
        startWith(LimelightReading(receiptTimestampMs = 1L, ageMs = 0L, pipelineIndex = 0, limelightTimestampMs = 100.0))
        assertTrue(subsystem.newFrameThisTick)

        // A new poll re-parses the same device frame: new receipt time, same ts.
        source.reading = source.reading.copy(receiptTimestampMs = 2L)
        subsystem.periodic()
        assertFalse(subsystem.newFrameThisTick)

        source.reading = source.reading.copy(receiptTimestampMs = 3L, limelightTimestampMs = 111.0)
        subsystem.periodic()
        assertTrue(subsystem.newFrameThisTick)
        assertEquals(2L, subsystem.receivedFrameCount)
    }

    @Test
    fun duplicatePollsExpireTargetsWithoutRenewingCaptureAge() {
        startWith(
            LimelightReading(
                receiptTimestampMs = 1000L, ageMs = 20L, valid = true, pipelineIndex = 0,
                limelightTimestampMs = 5000.0, captureLatencyMs = 11.0, targetingLatencyMs = 7.0,
                colorTargets = listOf(LimelightColorTarget(2.0, 3.0, 4.0)),
                fiducials = listOf(fiducial(34)),
            ),
        )

        repeat(7) {
            clock.advanceMs(10.0)
            source.reading = source.reading.copy(receiptTimestampMs = 2000L + it, ageMs = 0L)
            subsystem.periodic()
            assertTrue(subsystem.resultFresh)
            assertFalse(subsystem.newFrameThisTick)
        }
        clock.advanceMs(10.0)
        source.reading = source.reading.copy(receiptTimestampMs = 9000L, ageMs = 0L)
        subsystem.periodic()

        assertEquals(0L, subsystem.resultAgeMs)
        assertEquals(118.0, subsystem.estimatedCaptureAgeMs, 1e-9)
        assertFalse(subsystem.resultFresh)
        assertFalse(subsystem.targetVisible)
        assertNull(subsystem.primaryTarget)
        assertTrue(subsystem.colorTargets.isEmpty())
        assertTrue(subsystem.fiducials.isEmpty())
        assertEquals(1L, subsystem.receivedFrameCount)

        clock.advanceMs(500.0)
        source.reading = source.reading.copy(receiptTimestampMs = 9500L)
        subsystem.periodic()
        assertFalse(subsystem.resultFresh)
        assertEquals(618.0, subsystem.estimatedCaptureAgeMs, 1e-9)

        // A reboot can move the device clock backwards; a different ts is still a new frame.
        source.reading = source.reading.copy(limelightTimestampMs = 1.0)
        subsystem.periodic()
        assertTrue(subsystem.newFrameThisTick)
        assertTrue(subsystem.resultFresh)
        assertEquals(listOf(34), subsystem.fiducials.map { it.id })
        assertEquals(18.0, subsystem.estimatedCaptureAgeMs, 1e-9)
    }

    @Test
    fun reconnectingWithTheSameDeviceFrameDoesNotReviveIt() {
        startWith(
            LimelightReading(
                receiptTimestampMs = 1L, ageMs = 0L, valid = true, pipelineIndex = 0,
                limelightTimestampMs = 100.0, fiducials = listOf(fiducial(34)),
            ),
        )
        source.isConnected = false
        clock.advanceMs(500.0)
        subsystem.periodic()
        source.isConnected = true
        source.reading = source.reading.copy(receiptTimestampMs = 501L)
        subsystem.periodic()

        assertFalse(subsystem.newFrameThisTick)
        assertFalse(subsystem.resultFresh)
        assertTrue(subsystem.fiducials.isEmpty())
    }

    @Test
    fun frameWithoutDeviceTimestampStillExpiresBetweenReceipts() {
        startWith(LimelightReading(receiptTimestampMs = 1L, ageMs = 0L, valid = true, pipelineIndex = 0))
        clock.advanceMs(100.0)
        subsystem.periodic()
        assertFalse(subsystem.resultFresh)

        source.reading = source.reading.copy(receiptTimestampMs = 101L)
        subsystem.periodic()
        assertTrue(subsystem.newFrameThisTick)
        assertTrue(subsystem.resultFresh)
    }

    @Test
    fun missingResultHasInfiniteEstimatedAge() {
        assertTrue(subsystem.estimatedCaptureAgeMs.isInfinite())
    }

    @Test
    fun stopStopsPolling() {
        subsystem.init(HardwareMap(null, null))
        subsystem.stop()

        assertFalse(source.isRunning)
        assertFalse(subsystem.isRunning)
    }

    private fun fiducial(id: Int) = LimelightFiducial(
        id = id, family = "36H11C", txDegrees = 1.0, tyDegrees = 2.0,
        txNoCrosshairDegrees = 1.0, tyNoCrosshairDegrees = 2.0, areaPercent = 0.5,
        targetPoseCameraSpace = LimelightPose(0.1, -0.2, 1.0, 0.0, 5.0, 0.0),
        cameraPoseTargetSpace = null, cornerCount = 4,
    )

    private fun startWith(reading: LimelightReading) {
        source.reading = reading
        subsystem.init(HardwareMap(null, null))
        source.isConnected = true
        subsystem.periodic()
    }

    private class FakeLimelightSource : LimelightSource {
        override var isRunning = false
        override var isConnected = false
        var configuredPollRateHz = -1
        var configuredPipelineIndex = -1
        var reading = LimelightReading()

        override fun setPollRateHz(rateHz: Int) {
            configuredPollRateHz = rateHz
        }

        override fun pipelineSwitch(index: Int): Boolean {
            configuredPipelineIndex = index
            return true
        }

        override fun start() {
            isRunning = true
        }

        override fun latestReading(): LimelightReading = reading

        override fun stop() {
            isRunning = false
        }
    }
}
