package org.firstinspires.ftc.teamcode.vision.ball

import org.firstinspires.ftc.teamcode.core.util.Clock

enum class BallFrameStatus { NO_FRAME_YET, FRESH, STALE, PROCESSING_ERROR }

enum class BallTargetStatus {
    /** A fresh frame with an accepted candidate. */
    SELECTED,

    /** A fresh frame in which nothing passed the filters. */
    NONE_ACCEPTED,

    /** The newest frame is too old or failed; any earlier target is withdrawn. */
    CLEARED,

    NO_DATA,
}

/**
 * One robot-loop view of the ball camera. Candidates and the selected target
 * are present only while [frameStatus] is [BallFrameStatus.FRESH].
 */
data class BallObservation(
    val frameStatus: BallFrameStatus,
    val targetStatus: BallTargetStatus,
    val frame: BallFrameResult?,
    /** True on the first robot tick that sees [frame]. */
    val newFrame: Boolean,
    /** Robot-loop nanoTime of the tick that first saw [frame]. */
    val receiptNanos: Long,
    /** now − capture time; falls back to now − publish time when [captureTimeValid] is false. */
    val ageMs: Double,
    val captureTimeValid: Boolean,
    /** Camera capture → processor publish. */
    val captureToPublishMs: Double,
    /** Processor publish → robot receipt. */
    val publishToReceiptMs: Double,
    val selected: BallCandidate?,
    val candidates: List<BallCandidate>,
) {
    companion object {
        val NONE = BallObservation(
            BallFrameStatus.NO_FRAME_YET, BallTargetStatus.NO_DATA, null, false, 0L,
            Double.POSITIVE_INFINITY, false, Double.NaN, Double.NaN, null, emptyList(),
        )
    }
}

/**
 * Turns the camera thread's latest published [BallFrameResult] into a
 * [BallObservation] on the robot thread, and measures throughput honestly:
 *
 *  - [processedFps]: processor frames per second, from frame numbers and
 *    publish times — what the detector actually delivered, independent of how
 *    often the robot looks.
 *  - [receivedFps]: distinct frames the robot loop saw per second.
 *  - [skippedFrames]: processed frames the robot never saw because a newer one
 *    replaced them between ticks.
 *  - [repeatTicks]: robot ticks that found no new frame.
 *
 * None of these is the camera's advertised frame rate.
 */
class BallObservationTracker(private val clock: Clock) {

    private var lastFrameNumber = 0L
    private var lastReceiptNanos = 0L

    private var windowStartNanos = Long.MIN_VALUE
    private var windowReceived = 0L
    private var windowFirstFrame: BallFrameResult? = null

    var processedFps = 0.0
        private set
    var receivedFps = 0.0
        private set
    var receivedFrames = 0L
        private set
    var skippedFrames = 0L
        private set
    var repeatTicks = 0L
        private set
    var staleTicks = 0L
        private set
    var errorFrames = 0L
        private set

    var observation: BallObservation = BallObservation.NONE
        private set

    fun update(latest: BallFrameResult?, maxAgeMs: Double): BallObservation {
        val now = clock.nanos()
        if (windowStartNanos == Long.MIN_VALUE) windowStartNanos = now

        if (latest == null) {
            observation = BallObservation.NONE
            updateRates(now)
            return observation
        }

        val isNew = latest.frameNumber != lastFrameNumber
        if (isNew) {
            if (lastFrameNumber != 0L && latest.frameNumber > lastFrameNumber + 1) {
                skippedFrames += latest.frameNumber - lastFrameNumber - 1
            }
            lastFrameNumber = latest.frameNumber
            lastReceiptNanos = now
            receivedFrames++
            windowReceived++
            if (windowFirstFrame == null) windowFirstFrame = latest
            if (latest.error != null) errorFrames++
        } else {
            repeatTicks++
        }

        val captureValid = latest.captureTimeNanos in 1..latest.publishedNanos
        val reference = if (captureValid) latest.captureTimeNanos else latest.publishedNanos
        val ageMs = (now - reference) / 1e6

        val frameStatus = when {
            latest.error != null -> BallFrameStatus.PROCESSING_ERROR
            !(ageMs <= maxAgeMs) -> BallFrameStatus.STALE
            else -> BallFrameStatus.FRESH
        }
        if (frameStatus != BallFrameStatus.FRESH) staleTicks++

        val fresh = frameStatus == BallFrameStatus.FRESH
        val selection = latest.selection
        observation = BallObservation(
            frameStatus = frameStatus,
            targetStatus = when {
                !fresh -> BallTargetStatus.CLEARED
                selection.selected != null -> BallTargetStatus.SELECTED
                else -> BallTargetStatus.NONE_ACCEPTED
            },
            frame = latest,
            newFrame = isNew,
            receiptNanos = lastReceiptNanos,
            ageMs = ageMs,
            captureTimeValid = captureValid,
            captureToPublishMs = if (captureValid) (latest.publishedNanos - latest.captureTimeNanos) / 1e6 else Double.NaN,
            publishToReceiptMs = (lastReceiptNanos - latest.publishedNanos) / 1e6,
            selected = if (fresh) selection.selected else null,
            candidates = if (fresh) selection.candidates else emptyList(),
        )
        updateRates(now, latest)
        return observation
    }

    fun reset() {
        lastFrameNumber = 0L
        lastReceiptNanos = 0L
        windowStartNanos = Long.MIN_VALUE
        windowReceived = 0L
        windowFirstFrame = null
        processedFps = 0.0
        receivedFps = 0.0
        receivedFrames = 0L
        skippedFrames = 0L
        repeatTicks = 0L
        staleTicks = 0L
        errorFrames = 0L
        observation = BallObservation.NONE
    }

    private fun updateRates(now: Long, latest: BallFrameResult? = null) {
        val elapsed = now - windowStartNanos
        if (elapsed < RATE_WINDOW_NANOS) return
        receivedFps = windowReceived * 1e9 / elapsed
        val first = windowFirstFrame
        processedFps = if (first != null && latest != null && latest.publishedNanos > first.publishedNanos) {
            (latest.frameNumber - first.frameNumber) * 1e9 / (latest.publishedNanos - first.publishedNanos)
        } else {
            0.0
        }
        windowStartNanos = now
        windowReceived = 0L
        windowFirstFrame = latest
    }

    private companion object {
        const val RATE_WINDOW_NANOS = 1_000_000_000L
    }
}
