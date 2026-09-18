package org.firstinspires.ftc.teamcode.vision.ball

import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Test

class BallObservationTrackerTest {

    private val clock = FakeClock(start = 10_000_000_000L)
    private val tracker = BallObservationTracker(clock)
    private val filters = BlobFilterSettings(0.01, 50.0, 0.0, 10.0, 0.0, 8)

    @Test
    fun noFrameYetHasNoTarget() {
        val obs = tracker.update(null, maxAgeMs = 150.0)
        assertEquals(BallFrameStatus.NO_FRAME_YET, obs.frameStatus)
        assertEquals(BallTargetStatus.NO_DATA, obs.targetStatus)
        assertNull(obs.selected)
    }

    @Test
    fun freshFrameSeparatesCaptureProcessingAndReceiptTimes() {
        val frame = frame(number = 1, captureAgoMs = 40.0, processingMs = 12.0, publishedAgoMs = 5.0)
        val obs = tracker.update(frame, maxAgeMs = 150.0)

        assertEquals(BallFrameStatus.FRESH, obs.frameStatus)
        assertEquals(BallTargetStatus.SELECTED, obs.targetStatus)
        assertTrue(obs.newFrame)
        assertTrue(obs.captureTimeValid)
        assertEquals(40.0, obs.ageMs, 1e-6)
        assertEquals(35.0, obs.captureToPublishMs, 1e-6)
        assertEquals(12.0, frame.processingMs, 1e-6)
        assertEquals(5.0, obs.publishToReceiptMs, 1e-6)
        assertEquals(clock.now, obs.receiptNanos)
        assertSame(frame.selection.selected, obs.selected)
    }

    @Test
    fun staleFrameClearsSelectedTargetAndCandidates() {
        val frame = frame(number = 1, captureAgoMs = 20.0)
        assertEquals(BallTargetStatus.SELECTED, tracker.update(frame, 150.0).targetStatus)

        clock.advanceMs(200.0)
        val obs = tracker.update(frame, 150.0)

        assertEquals(BallFrameStatus.STALE, obs.frameStatus)
        assertEquals(BallTargetStatus.CLEARED, obs.targetStatus)
        assertFalse(obs.newFrame)
        assertNull(obs.selected)
        assertTrue(obs.candidates.isEmpty())
        assertEquals(1, tracker.staleTicks)
        assertEquals(1, tracker.repeatTicks)
    }

    @Test
    fun processingErrorFrameIsNeverFresh() {
        val obs = tracker.update(frame(number = 1, captureAgoMs = 1.0).copy(error = "boom"), 150.0)
        assertEquals(BallFrameStatus.PROCESSING_ERROR, obs.frameStatus)
        assertEquals(BallTargetStatus.CLEARED, obs.targetStatus)
        assertEquals(1, tracker.errorFrames)
    }

    @Test
    fun freshFrameWithoutAcceptedBlobReportsNoneAccepted() {
        val empty = frame(number = 1, captureAgoMs = 1.0).copy(selection = CandidateSelection.EMPTY)
        assertEquals(BallTargetStatus.NONE_ACCEPTED, tracker.update(empty, 150.0).targetStatus)
    }

    @Test
    fun invalidCaptureTimeFallsBackToPublishTimeAndSaysSo() {
        val frame = frame(number = 1, captureAgoMs = 0.0, publishedAgoMs = 30.0).copy(captureTimeNanos = 0L)
        val obs = tracker.update(frame, 150.0)
        assertFalse(obs.captureTimeValid)
        assertEquals(30.0, obs.ageMs, 1e-6)
        assertTrue(obs.captureToPublishMs.isNaN())
    }

    @Test
    fun countsSkippedFramesAndMeasuresRatesFromFrameIdentity() {
        // The processor advances two frame numbers per 25 ms of publish time;
        // the robot looks every 25 ms, so it sees every other processed frame.
        var number = 0L
        repeat(60) {
            clock.advanceMs(25.0)
            number += 2
            tracker.update(frame(number, captureAgoMs = 2.0, publishedAgoMs = 1.0), 150.0)
        }

        assertEquals(60, tracker.receivedFrames)
        assertEquals(59, tracker.skippedFrames)
        assertEquals(41.0, tracker.receivedFps, 1e-6)
        assertEquals(80.0, tracker.processedFps, 1e-6)
    }

    @Test
    fun ratesFallToZeroWhenFramesStop() {
        val frame = frame(number = 5, captureAgoMs = 1.0)
        repeat(50) {
            tracker.update(frame, 10_000.0)
            clock.advanceMs(25.0)
        }
        assertEquals(0.0, tracker.processedFps, 0.0)
        assertEquals(1.0, tracker.receivedFps, 1e-6)
    }

    @Test
    fun resetStartsCleanForARestartedCamera() {
        tracker.update(frame(number = 9, captureAgoMs = 1.0), 150.0)
        tracker.reset()
        val obs = tracker.update(frame(number = 1, captureAgoMs = 1.0), 150.0)
        assertTrue(obs.newFrame)
        assertEquals(0, tracker.skippedFrames)
    }

    private fun frame(
        number: Long,
        captureAgoMs: Double,
        processingMs: Double = 5.0,
        publishedAgoMs: Double = 0.0,
    ): BallFrameResult {
        val published = clock.now - (publishedAgoMs * 1e6).toLong()
        return BallFrameResult(
            frameNumber = number,
            captureTimeNanos = clock.now - (captureAgoMs * 1e6).toLong(),
            processingStartNanos = published - (processingMs * 1e6).toLong(),
            publishedNanos = published,
            widthPx = 640,
            heightPx = 480,
            roi = PixelRect(0, 0, 640, 480),
            settingsVersion = 1,
            contourCount = 1,
            ignoredSmallCount = 0,
            selection = BallCandidateFilter.evaluate(listOf(BallCandidateFilterTest.blob()), 640.0 * 480, filters),
        )
    }
}
