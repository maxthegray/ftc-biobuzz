package org.firstinspires.ftc.teamcode.vision.ball

/**
 * Shape measurements of one color contour. Image coordinates are pixels of the
 * full processed frame (not the ROI): origin top-left, +x right, +y down.
 */
data class BlobMeasurement(
    /** Area-weighted contour centroid. */
    val centroidXPx: Double,
    val centroidYPx: Double,
    /** Radius of the minimum enclosing circle. */
    val enclosingRadiusPx: Double,
    val contourAreaPx: Double,
    val boxLeftPx: Int,
    val boxTopPx: Int,
    val boxWidthPx: Int,
    val boxHeightPx: Int,
    val circularity: Double,
    val aspectRatio: Double,
    val density: Double,
)

enum class BlobRejection(val label: String) {
    TOO_SMALL("small"),
    TOO_LARGE("large"),
    NOT_CIRCULAR("circularity"),
    ELONGATED("aspect"),
    NOT_SOLID("density"),
}

data class BallCandidate(val blob: BlobMeasurement, val rejection: BlobRejection?) {
    val accepted: Boolean get() = rejection == null
}

/**
 * The filter outcome for one frame. [candidates] lists accepted blobs first,
 * each group largest-first, truncated to the publish limit; [selected] is
 * decided before truncation.
 */
data class CandidateSelection(
    val candidates: List<BallCandidate>,
    val selected: BallCandidate?,
    val evaluatedCount: Int,
    val acceptedCount: Int,
) {
    companion object {
        val EMPTY = CandidateSelection(emptyList(), null, 0, 0)
    }
}

/**
 * Blob filtering and target selection, free of OpenCV so it runs in host tests.
 *
 * Selection rule: the accepted blob with the largest contour area. Ties go to
 * the blob lower in the image (larger y), then further left, so the choice is
 * deterministic. It is a diagnostic rule — nothing here claims the largest
 * blob is the nearest ball.
 */
object BallCandidateFilter {

    fun evaluate(
        blobs: List<BlobMeasurement>,
        frameAreaPx: Double,
        filters: BlobFilterSettings,
    ): CandidateSelection {
        if (blobs.isEmpty()) return CandidateSelection.EMPTY
        val minArea = frameAreaPx * filters.minAreaPercent / 100.0
        val maxArea = frameAreaPx * filters.maxAreaPercent / 100.0

        val evaluated = blobs.map { BallCandidate(it, rejectionOf(it, minArea, maxArea, filters)) }
        val ordered = evaluated.sortedWith(
            compareBy<BallCandidate> { !it.accepted }
                .thenByDescending { it.blob.contourAreaPx }
                .thenByDescending { it.blob.centroidYPx }
                .thenBy { it.blob.centroidXPx },
        )
        val selected = ordered.firstOrNull()?.takeIf { it.accepted }
        return CandidateSelection(
            candidates = ordered.take(filters.maxPublishedCandidates),
            selected = selected,
            evaluatedCount = evaluated.size,
            acceptedCount = evaluated.count { it.accepted },
        )
    }

    /** Checks run in this order; the first failure is the one reported. */
    fun rejectionOf(
        blob: BlobMeasurement,
        minAreaPx: Double,
        maxAreaPx: Double,
        filters: BlobFilterSettings,
    ): BlobRejection? = when {
        !(blob.contourAreaPx >= minAreaPx) -> BlobRejection.TOO_SMALL
        blob.contourAreaPx > maxAreaPx -> BlobRejection.TOO_LARGE
        !(blob.circularity >= filters.minCircularity) -> BlobRejection.NOT_CIRCULAR
        !(blob.aspectRatio <= filters.maxAspectRatio) -> BlobRejection.ELONGATED
        !(blob.density >= filters.minDensity) -> BlobRejection.NOT_SOLID
        else -> null
    }
}

/**
 * Everything one processed camera frame produced, published atomically by the
 * camera thread.
 *
 * Timestamps share the `System.nanoTime()` clock. [captureTimeNanos] is the SDK
 * `CameraFrame.getCaptureTime()` value handed to the processor — documented as
 * the capture time, stamped in the SDK's native UVC layer; how it relates to
 * sensor exposure is not documented, so treat it as the best available
 * acquisition estimate. [processingStartNanos] and [publishedNanos] bracket
 * this code's work.
 */
data class BallFrameResult(
    /** Processor-local sequence number: 1 for the first processed frame, +1 per frame. */
    val frameNumber: Long,
    val captureTimeNanos: Long,
    val processingStartNanos: Long,
    val publishedNanos: Long,
    val widthPx: Int,
    val heightPx: Int,
    val roi: PixelRect,
    val settingsVersion: Long,
    val contourCount: Int,
    /** Contours below half the minimum area, dropped before shape measurement. */
    val ignoredSmallCount: Int,
    val selection: CandidateSelection,
    /** Non-null when this frame's processing threw; [selection] is then empty. */
    val error: String? = null,
) {
    val processingMs: Double get() = (publishedNanos - processingStartNanos) / 1e6
}
