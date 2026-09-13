package org.firstinspires.ftc.teamcode.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Test

class BallCandidateFilterTest {

    private val frameArea = 640.0 * 480.0 // 307200 px²; 0.1 % = 307.2 px²
    private val filters = BlobFilterSettings(
        minAreaPercent = 0.1,
        maxAreaPercent = 10.0,
        minCircularity = 0.6,
        maxAspectRatio = 1.5,
        minDensity = 0.8,
        maxPublishedCandidates = 8,
    )

    @Test
    fun selectsLargestAcceptedBlob() {
        val small = blob(area = 400.0, x = 100.0)
        val large = blob(area = 900.0, x = 300.0)
        val selection = BallCandidateFilter.evaluate(listOf(small, large), frameArea, filters)

        assertSame(large, selection.selected!!.blob)
        assertEquals(2, selection.acceptedCount)
        assertEquals(listOf(large, small), selection.candidates.map { it.blob })
    }

    @Test
    fun largerRejectedBlobNeverWinsAndReportsFirstFailedCheck() {
        val elongated = blob(area = 5000.0, aspect = 3.0)
        val good = blob(area = 500.0)
        val selection = BallCandidateFilter.evaluate(listOf(elongated, good), frameArea, filters)

        assertSame(good, selection.selected!!.blob)
        assertEquals(BlobRejection.ELONGATED, selection.candidates[1].rejection)
    }

    @Test
    fun eachFilterRejectsWithItsOwnReason() {
        val minArea = frameArea * filters.minAreaPercent / 100
        val maxArea = frameArea * filters.maxAreaPercent / 100
        fun reason(b: BlobMeasurement) = BallCandidateFilter.rejectionOf(b, minArea, maxArea, filters)

        assertEquals(BlobRejection.TOO_SMALL, reason(blob(area = 300.0)))
        assertEquals(BlobRejection.TOO_LARGE, reason(blob(area = 40000.0)))
        assertEquals(BlobRejection.NOT_CIRCULAR, reason(blob(circularity = 0.4)))
        assertEquals(BlobRejection.ELONGATED, reason(blob(aspect = 1.6)))
        assertEquals(BlobRejection.NOT_SOLID, reason(blob(density = 0.5)))
        assertEquals(BlobRejection.NOT_CIRCULAR, reason(blob(circularity = Double.NaN)))
        assertNull(reason(blob()))
    }

    @Test
    fun noAcceptedCandidateMeansNoSelection() {
        val selection = BallCandidateFilter.evaluate(listOf(blob(circularity = 0.1)), frameArea, filters)
        assertNull(selection.selected)
        assertEquals(1, selection.candidates.size)
        assertEquals(0, selection.acceptedCount)
    }

    @Test
    fun equalAreasBreakTiesLowerInImageThenLeft() {
        val high = blob(area = 600.0, x = 50.0, y = 100.0)
        val lowRight = blob(area = 600.0, x = 400.0, y = 300.0)
        val lowLeft = blob(area = 600.0, x = 200.0, y = 300.0)
        val selection = BallCandidateFilter.evaluate(listOf(high, lowRight, lowLeft), frameArea, filters)
        assertSame(lowLeft, selection.selected!!.blob)
    }

    @Test
    fun publishLimitTruncatesButSelectionSeesEverything() {
        val blobs = (1..5).map { blob(area = 400.0 + it) }
        val selection = BallCandidateFilter.evaluate(blobs, frameArea, filters.copy(maxPublishedCandidates = 2))

        assertEquals(2, selection.candidates.size)
        assertEquals(5, selection.evaluatedCount)
        assertEquals(405.0, selection.selected!!.blob.contourAreaPx, 0.0)
    }

    @Test
    fun emptyInputIsEmptySelection() {
        assertSame(CandidateSelection.EMPTY, BallCandidateFilter.evaluate(emptyList(), frameArea, filters))
    }

    companion object {
        fun blob(
            area: Double = 500.0,
            x: Double = 320.0,
            y: Double = 240.0,
            circularity: Double = 0.85,
            aspect: Double = 1.1,
            density: Double = 0.95,
        ) = BlobMeasurement(
            centroidXPx = x, centroidYPx = y, enclosingRadiusPx = Math.sqrt(area / Math.PI),
            contourAreaPx = area, boxLeftPx = 0, boxTopPx = 0, boxWidthPx = 10, boxHeightPx = 10,
            circularity = circularity, aspectRatio = aspect, density = density,
        )
    }
}
