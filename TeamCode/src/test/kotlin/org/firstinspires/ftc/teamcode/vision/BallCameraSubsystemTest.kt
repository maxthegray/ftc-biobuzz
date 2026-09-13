package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.runtime.HardwareConfigError
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class BallCameraSubsystemTest {

    private val clock = FakeClock()
    private val opened = ArrayList<FakeBackend>()
    private val factory = BallCameraBackendFactory { _, name, stream, detection ->
        FakeBackend(name, stream, detection).also { opened += it }
    }

    @Before
    fun setUp() = BallVisionConfig.resetDefaults()

    @After
    fun tearDown() = BallVisionConfig.resetDefaults()

    private fun subsystem() = BallCameraSubsystem(backendFactory = factory, clock = clock)

    @Test
    fun initOpensOnceWithStartupStreamAndPublishesInitialSettings() {
        val camera = subsystem()
        camera.init(HardwareMap(null, null))

        assertEquals(1, opened.size)
        val backend = opened.single()
        assertEquals("ballCamera", backend.name)
        assertEquals(StreamSettings(640, 480, BallStreamFormat.MJPEG), backend.stream)
        assertEquals(listOf(1L), backend.published.map { it.second })
        assertEquals(1, backend.cameraRequests.size)
    }

    @Test
    fun periodicPublishesOnlyChangedSettingsAndBumpsTheVersion() {
        val camera = subsystem()
        camera.init(HardwareMap(null, null))
        val backend = opened.single()

        repeat(3) { camera.periodic() }
        assertEquals(1, backend.published.size)
        assertEquals(1, backend.cameraRequests.size)

        BallVisionConfig.minCircularity = 0.8
        camera.periodic()
        assertEquals(2, backend.published.size)
        assertEquals(2L, camera.settingsVersion)
        assertEquals(0.8, backend.published.last().first.filters.minCircularity, 0.0)
        assertEquals(1, backend.cameraRequests.size)

        BallVisionConfig.whiteBalanceManual = true
        camera.periodic()
        assertEquals(2, backend.published.size)
        assertEquals(2, backend.cameraRequests.size)
    }

    @Test
    fun tuningValuesReachTheFlightLogAtInitAndOnChange() {
        val events = ArrayList<String>()
        val camera = BallCameraSubsystem(backendFactory = factory, clock = clock, eventSink = { events += it })
        camera.init(HardwareMap(null, null))
        assertEquals(1, events.size)
        assertTrue(events[0].startsWith("ballVision (detection v1): colorSpace=0 channel0Min=32"))

        repeat(3) { camera.periodic() }
        assertEquals(1, events.size)

        clock.advanceMs(1500.0)
        BallVisionConfig.channel1Min = 140
        camera.periodic()
        assertEquals("ballVision changed (detection v2): channel1Min 128→140", events.last())

        BallVisionConfig.exposureManual = true
        camera.periodic()
        assertEquals(2, events.size)
        clock.advanceMs(1100.0)
        camera.periodic()
        assertEquals("ballVision changed (detection v2): exposureManual false→true", events.last())
    }

    @Test
    fun previewToggleReachesBothTheProcessorAndTheLiveView() {
        val camera = subsystem()
        camera.init(HardwareMap(null, null))
        BallVisionConfig.previewEnabled = false
        camera.periodic()

        val backend = opened.single()
        assertFalse(backend.published.last().first.previewEnabled)
        assertFalse(backend.cameraRequests.last().liveViewEnabled)
    }

    @Test
    fun restartOnlySettingsAreReportedNotApplied() {
        val camera = subsystem()
        camera.init(HardwareMap(null, null))
        BallVisionConfig.resolutionWidth = 320
        BallVisionConfig.resolutionHeight = 240
        camera.periodic()

        assertTrue(camera.restartRequired)
        assertEquals(StreamSettings(640, 480, BallStreamFormat.MJPEG), camera.runningStream)
        assertEquals(1, opened.size)
        assertTrue(camera.health().startsWith("restart required"))
    }

    @Test
    fun unsupportedStreamModeFailsInitBeforeOpeningTheCamera() {
        BallVisionConfig.streamFormat = BallVisionConfig.STREAM_YUY2
        try {
            subsystem().init(HardwareMap(null, null))
            error("expected HardwareConfigError")
        } catch (e: HardwareConfigError) {
            assertTrue(e.message!!.contains("640x480 YUY2"))
        }
        assertTrue(opened.isEmpty())
    }

    @Test
    fun staleBackendFrameIsClearedByTheSubsystem() {
        val camera = subsystem()
        camera.init(HardwareMap(null, null))
        val backend = opened.single()
        backend.frame = frameCapturedAt(clock.now - 10_000_000L)
        camera.periodic()
        assertEquals(BallTargetStatus.SELECTED, camera.observation.targetStatus)

        clock.advanceMs(500.0)
        camera.periodic()
        assertEquals(BallTargetStatus.CLEARED, camera.observation.targetStatus)
        assertNull(camera.observation.selected)
    }

    @Test
    fun resetToDefaultsFlagIsHandledOnTheRobotLoop() {
        val camera = subsystem()
        camera.init(HardwareMap(null, null))
        BallVisionConfig.channel0Min = 99
        BallVisionConfig.resetToDefaults = true
        camera.periodic()
        assertEquals(32, BallVisionConfig.channel0Min)
        assertFalse(BallVisionConfig.resetToDefaults)
        assertEquals(32, opened.single().published.last().first.lower.c0)
    }

    @Test
    fun stopClosesOnceWithTimeoutAndIsSafeBeforeInitOrTwice() {
        subsystem().stop()

        val camera = subsystem()
        camera.init(HardwareMap(null, null))
        camera.stop()
        camera.stop()
        camera.periodic()

        val backend = opened.single()
        assertEquals(listOf(BallCameraSubsystem.CLOSE_TIMEOUT_MS), backend.closeCalls)
        assertTrue(camera.closed)
        assertEquals("CLOSED", camera.cameraState)
    }

    @Test
    fun closeTimeoutAndExceptionsNeverEscapeStop() {
        val camera = subsystem()
        camera.init(HardwareMap(null, null))
        opened.single().closeResult = false
        camera.stop()
        assertTrue(camera.closeTimedOut)

        val throwing = subsystem()
        throwing.init(HardwareMap(null, null))
        opened.last().throwOnClose = true
        throwing.stop()
        assertTrue(throwing.closed)
    }

    @Test
    fun repeatedOpModeRunsGetFreshBackendsAndTrackers() {
        val first = subsystem()
        first.init(HardwareMap(null, null))
        opened.last().frame = frameCapturedAt(clock.now)
        first.periodic()
        first.stop()

        val second = subsystem()
        second.init(HardwareMap(null, null))
        second.periodic()

        assertEquals(2, opened.size)
        assertEquals(BallFrameStatus.NO_FRAME_YET, second.observation.frameStatus)
        assertSame(opened[1].stream, second.runningStream)
    }

    private fun frameCapturedAt(captureNanos: Long): BallFrameResult {
        val filters = BlobFilterSettings(0.01, 50.0, 0.0, 10.0, 0.0, 8)
        return BallFrameResult(
            frameNumber = 1, captureTimeNanos = captureNanos, processingStartNanos = captureNanos,
            publishedNanos = captureNanos + 1, widthPx = 640, heightPx = 480, roi = PixelRect(0, 0, 640, 480),
            settingsVersion = 1, contourCount = 1, ignoredSmallCount = 0,
            selection = BallCandidateFilter.evaluate(listOf(BallCandidateFilterTest.blob()), 640.0 * 480, filters),
        )
    }

    private class FakeBackend(
        val name: String,
        val stream: StreamSettings,
        detection: DetectionSettings,
    ) : BallCameraBackend {
        val published = ArrayList<Pair<DetectionSettings, Long>>()
        val cameraRequests = ArrayList<CameraImageRequest>()
        val closeCalls = ArrayList<Long>()
        var frame: BallFrameResult? = null
        var closeResult = true
        var throwOnClose = false

        override val cameraState = "STREAMING"
        override val libraryReportedFps = 0.0
        override val processorFaults = 0L
        override val lastProcessorError: String? = null

        override fun latestFrame() = frame
        override fun lensReport() = LensReport.NOT_YET_KNOWN
        override fun publishDetectionSettings(settings: DetectionSettings, version: Long) {
            published += settings to version
        }
        override fun requestCameraImage(request: CameraImageRequest) {
            cameraRequests += request
        }
        override fun controlStatus() = CameraControlStatus()
        override fun close(timeoutMs: Long): Boolean {
            closeCalls += timeoutMs
            if (throwOnClose) throw IllegalStateException("close")
            return closeResult
        }
    }
}
