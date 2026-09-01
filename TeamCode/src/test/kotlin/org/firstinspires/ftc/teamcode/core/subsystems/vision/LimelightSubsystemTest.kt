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
    fun stopStopsPolling() {
        subsystem.init(HardwareMap(null, null))
        subsystem.stop()

        assertFalse(source.isRunning)
        assertFalse(subsystem.isRunning)
    }

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
