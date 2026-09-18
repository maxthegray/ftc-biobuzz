package org.firstinspires.ftc.teamcode.vision.ball

import org.firstinspires.ftc.teamcode.vision.diagnostics.VisionDiagnosticsConfig
import org.firstinspires.ftc.teamcode.vision.diagnostics.VisionLabRecord
import java.lang.reflect.Modifier
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class BallVisionSettingsTest {

    @Before
    fun setUp() = BallVisionConfig.resetDefaults()

    @After
    fun tearDown() = BallVisionConfig.resetDefaults()

    @Test
    fun defaultsProduce640x480MjpegAndTentativeTunedSettings() {
        val s = BallVisionSettings.fromConfig()
        assertEquals(StreamSettings(640, 480, BallStreamFormat.MJPEG), s.stream)
        assertEquals(BallColorSpace.YCRCB, s.detection.colorSpace)
        assertEquals(ChannelTriple(125, 130, 50), s.detection.lower)
        assertEquals(ChannelTriple(255, 170, 110), s.detection.upper)
        assertEquals(1.0, s.detection.filters.minCircularity, 0.0)
        assertTrue(s.camera.exposureManual)
        assertEquals(6000L, s.camera.exposureMicros)
        assertEquals(NormalizedRoi.FULL_FRAME, s.detection.roi)
        assertTrue(s.stream in StreamSettings.GOBILDA_3122_0004_0001_MODES)
        assertTrue(StreamSettings(320, 240, BallStreamFormat.MJPEG) in StreamSettings.GOBILDA_3122_0004_0001_MODES)
    }

    @Test
    fun invalidPanelsInputIsRepairedNotPassedToTheCameraThread() {
        BallVisionConfig.channel1Min = 200
        BallVisionConfig.channel1Max = 100
        BallVisionConfig.channel2Max = 999
        BallVisionConfig.colorSpace = BallVisionConfig.COLOR_SPACE_HSV
        BallVisionConfig.channel0Max = 255
        BallVisionConfig.roiLeft = 0.8
        BallVisionConfig.roiRight = 0.2
        BallVisionConfig.blurKernelPx = 4
        BallVisionConfig.erodeKernelPx = -3
        BallVisionConfig.minAreaPercent = 5.0
        BallVisionConfig.maxAreaPercent = 1.0
        BallVisionConfig.minCircularity = Double.NaN
        BallVisionConfig.maxObservationAgeMs = -1.0
        BallVisionConfig.previewMode = 42

        val s = BallVisionSettings.fromConfig()
        assertEquals(100, s.detection.lower.c1)
        assertEquals(200, s.detection.upper.c1)
        assertEquals(255, s.detection.upper.c2)
        assertEquals("HSV hue is limited to OpenCV's 0–180", 180, s.detection.upper.c0)
        assertEquals(NormalizedRoi.FULL_FRAME, s.detection.roi)
        assertEquals(5, s.detection.blurKernelPx)
        assertEquals(0, s.detection.erodeKernelPx)
        assertEquals(5.0, s.detection.filters.maxAreaPercent, 0.0)
        assertEquals(0.0, s.detection.filters.minCircularity, 0.0)
        assertEquals(150.0, s.maxObservationAgeMs, 0.0)
        assertEquals(BallPreviewMode.OVERLAY, s.detection.previewMode)
    }

    @Test
    fun liveAndRestartSettingsChangeIndependently() {
        val before = BallVisionSettings.fromConfig()
        BallVisionConfig.exposureMicros = 2000
        val exposureOnly = BallVisionSettings.fromConfig()
        assertNotEquals(before.camera, exposureOnly.camera)
        assertEquals(before.detection, exposureOnly.detection)
        assertEquals(before.stream, exposureOnly.stream)

        BallVisionConfig.resolutionWidth = 320
        BallVisionConfig.resolutionHeight = 240
        val restart = BallVisionSettings.fromConfig()
        assertNotEquals(before.stream, restart.stream)
        assertEquals(exposureOnly.detection, restart.detection)
    }

    @Test
    fun roiMapsToPixelsInsideTheFrame() {
        assertEquals(PixelRect(160, 120, 320, 240), NormalizedRoi(0.25, 0.25, 0.75, 0.75).toPixels(640, 480))
        assertEquals(PixelRect(80, 60, 160, 120), NormalizedRoi(0.25, 0.25, 0.75, 0.75).toPixels(320, 240))
        val sliver = NormalizedRoi(0.9999, 0.9999, 1.0, 1.0).toPixels(640, 480)
        assertTrue(sliver.width >= 1 && sliver.height >= 1)
        assertTrue(sliver.left + sliver.width <= 640 && sliver.top + sliver.height <= 480)
    }

    @Test
    fun resetToDefaultsFlagRestoresEveryFieldAndClearsItself() {
        BallVisionConfig.channel0Min = 1
        BallVisionConfig.exposureManual = false
        BallVisionConfig.resetToDefaults = true
        BallVisionConfig.resetDefaults()
        assertEquals(125, BallVisionConfig.channel0Min)
        assertTrue(BallVisionConfig.exposureManual)
        assertFalse(BallVisionConfig.resetToDefaults)
    }

    @Test
    fun compiledDefaultsListEveryTunableAndMatchTheConstantsLabRecordsNameForAdoption() {
        assertDefaultsConsistent(BallVisionConfig, BallVisionConfig.compiledDefaults()) { BallVisionConfig.resetDefaults() }
        assertDefaultsConsistent(VisionDiagnosticsConfig, VisionDiagnosticsConfig.compiledDefaults()) {
            VisionDiagnosticsConfig.resetDefaults()
        }
    }

    private fun assertDefaultsConsistent(config: Any, defaults: Map<String, Any>, reset: () -> Unit) {
        val tunables = config.javaClass.declaredFields.filter {
            Modifier.isPublic(it.modifiers) && !Modifier.isFinal(it.modifiers) &&
                (it.type.isPrimitive || it.type == String::class.java)
        }
        assertEquals(tunables.map { it.name }.toSet(), defaults.keys)
        reset()
        for (field in tunables) {
            assertEquals("reset value of ${field.name}", defaults[field.name], field.get(config))
            val constant = config.javaClass.getDeclaredField("DEFAULT_${VisionLabRecord.upperSnake(field.name)}")
            constant.isAccessible = true
            assertEquals("constant for ${field.name}", defaults[field.name], constant.get(null))
        }
    }
}
