package org.firstinspires.ftc.teamcode.vision.ball

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.BlobCriteria
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Before
import org.junit.Test

class BallCameraSettingsTest {

    @Before
    fun setUp() = BallCameraConfig.resetDefaults()

    @After
    fun tearDown() = BallCameraConfig.resetDefaults()

    @Test
    fun defaultsBuildTheTunedYCrCbRangeOverTheWholeFrame() {
        val setup = DetectionSetup.fromConfig(640, 480)
        assertEquals(false, setup.hsv)
        assertEquals(listOf(125, 130, 50), setup.min)
        assertEquals(listOf(255, 170, 110), setup.max)
        assertEquals(listOf(0, 0, 640, 480), setup.roiPx)
    }

    @Test
    fun invalidValuesAreClampedInsteadOfCrashingInit() {
        BallCameraConfig.colorSpace = BallCameraConfig.COLOR_SPACE_HSV
        BallCameraConfig.channel0Max = 255
        BallCameraConfig.channel1Min = -5
        BallCameraConfig.roiLeft = 0.8
        BallCameraConfig.roiRight = 0.2
        BallCameraConfig.blurKernelPx = -1
        val setup = DetectionSetup.fromConfig(640, 480)
        assertEquals(180, setup.max[0])
        assertEquals(0, setup.min[1])
        assertEquals(listOf(0, 0, 640, 480), setup.roiPx)
        assertEquals(0, setup.blurPx)
    }

    @Test
    fun roiFractionsBecomePixels() {
        BallCameraConfig.roiLeft = 0.25
        BallCameraConfig.roiTop = 0.5
        BallCameraConfig.roiRight = 0.75
        BallCameraConfig.roiBottom = 1.0
        assertEquals(listOf(160, 240, 480, 480), DetectionSetup.fromConfig(640, 480).roiPx)
    }

    @Test
    fun initSettingChangesAreDetectedButFilterChangesAreNot() {
        val before = DetectionSetup.fromConfig(640, 480)
        BallCameraConfig.minCircularity = 0.6
        assertEquals(before, DetectionSetup.fromConfig(640, 480))
        BallCameraConfig.channel1Min = 120
        assertNotEquals(before, DetectionSetup.fromConfig(640, 480))
    }

    @Test
    fun areaFiltersArePercentagesOfTheFrame() {
        val filters = blobFilters(640.0 * 480.0)
        val area = filters.single { it.criteria == BlobCriteria.BY_CONTOUR_AREA }
        assertEquals(153.6, area.minValue, 1e-9)
        assertEquals(122_880.0, area.maxValue, 1e-9)
        assertEquals(1.0, filters.single { it.criteria == BlobCriteria.BY_CIRCULARITY }.minValue, 0.0)
        assertEquals(2.5, filters.single { it.criteria == BlobCriteria.BY_ASPECT_RATIO }.maxValue, 0.0)
        assertEquals(0.7, filters.single { it.criteria == BlobCriteria.BY_DENSITY }.minValue, 0.0)
    }
}
