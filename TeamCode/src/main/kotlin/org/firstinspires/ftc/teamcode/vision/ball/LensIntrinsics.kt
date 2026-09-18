package org.firstinspires.ftc.teamcode.vision.ball

import kotlin.math.atan

/**
 * Lens intrinsics for one capture resolution, as the SDK resolved them from
 * `res/xml/teamwebcamcalibrations.xml` (or its built-in list) and passed to
 * `VisionProcessor.init`. This is the only path by which the ball processor
 * learns the calibration: nothing here reads AprilTag lens settings.
 *
 * Distortion coefficients use the SDK/OpenCV order
 * `[k1, k2, p1, p2, k3, k4, k5, k6]`.
 *
 * Lens only: camera height, pitch, and position on the robot are mounting
 * measurements and deliberately have no place here.
 */
data class LensIntrinsics(
    val widthPx: Int,
    val heightPx: Int,
    val fx: Double,
    val fy: Double,
    val cx: Double,
    val cy: Double,
    val distortion: List<Double>,
) {
    init {
        require(fx > 0.0 && fy > 0.0) { "focal lengths must be positive" }
    }

    private fun k(i: Int): Double = distortion.getOrElse(i) { 0.0 }

    /**
     * Undistorted normalized image coordinates for a pixel, via the same
     * fixed-point iteration OpenCV's `undistortPoints` uses.
     */
    fun normalizedUndistorted(xPx: Double, yPx: Double): Pair<Double, Double> {
        val x0 = (xPx - cx) / fx
        val y0 = (yPx - cy) / fy
        var x = x0
        var y = y0
        val k1 = k(0); val k2 = k(1); val p1 = k(2); val p2 = k(3)
        val k3 = k(4); val k4 = k(5); val k5 = k(6); val k6 = k(7)
        repeat(UNDISTORT_ITERATIONS) {
            val r2 = x * x + y * y
            val radial = (1 + ((k6 * r2 + k5) * r2 + k4) * r2) / (1 + ((k3 * r2 + k2) * r2 + k1) * r2)
            val dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
            val dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
            x = (x0 - dx) * radial
            y = (y0 - dy) * radial
        }
        return x to y
    }

    /** Forward model: normalized undistorted coordinates → pixel. Used to verify [normalizedUndistorted]. */
    fun distortToPixel(xNorm: Double, yNorm: Double): Pair<Double, Double> {
        val k1 = k(0); val k2 = k(1); val p1 = k(2); val p2 = k(3)
        val k3 = k(4); val k4 = k(5); val k5 = k(6); val k6 = k(7)
        val r2 = xNorm * xNorm + yNorm * yNorm
        val radial = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2)
        val xd = xNorm * radial + 2 * p1 * xNorm * yNorm + p2 * (r2 + 2 * xNorm * xNorm)
        val yd = yNorm * radial + p1 * (r2 + 2 * yNorm * yNorm) + 2 * p2 * xNorm * yNorm
        return (xd * fx + cx) to (yd * fy + cy)
    }

    /**
     * Camera-relative ray angles in degrees, each measured in its own camera
     * plane: horizontal in x–z (+ right of the optical axis), vertical in y–z
     * (+ below it). These are angles from the lens's principal point, not from
     * the robot and not from the image centre.
     */
    fun rayAnglesDegrees(xPx: Double, yPx: Double): Pair<Double, Double> {
        val (x, y) = normalizedUndistorted(xPx, yPx)
        return Math.toDegrees(atan(x)) to Math.toDegrees(atan(y))
    }

    private companion object {
        const val UNDISTORT_ITERATIONS = 10
    }
}

enum class CalibrationSource { EXACT, SCALED, UNAVAILABLE, NOT_YET_KNOWN }

/** What the SDK supplied for the running resolution, reported for the lab to verify on hardware. */
data class LensReport(
    val source: CalibrationSource,
    val intrinsics: LensIntrinsics?,
    /** Resolution of the calibration entry the SDK scaled from, when [source] is [CalibrationSource.SCALED]. */
    val scaledFrom: String? = null,
) {
    fun describe(): String = when (source) {
        CalibrationSource.NOT_YET_KNOWN -> "waiting for first frame"
        CalibrationSource.UNAVAILABLE -> "none for this camera/resolution — angles unavailable"
        CalibrationSource.EXACT -> "exact ${intrinsics?.widthPx}x${intrinsics?.heightPx} entry"
        CalibrationSource.SCALED -> "scaled from $scaledFrom"
    }

    companion object {
        val NOT_YET_KNOWN = LensReport(CalibrationSource.NOT_YET_KNOWN, null)
    }
}
