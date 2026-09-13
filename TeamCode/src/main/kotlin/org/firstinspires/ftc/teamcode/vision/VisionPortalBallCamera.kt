package org.firstinspires.ftc.teamcode.vision

import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.concurrent.TimeUnit
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.teamcode.core.runtime.HardwareConfigError
import org.firstinspires.ftc.vision.VisionPortal

/**
 * [BallCameraBackend] over an SDK 11.1.0 VisionPortal with the live view in
 * the Robot Controller's default camera-monitor container (visible on Control
 * Hub HDMI or scrcpy).
 */
class VisionPortalBallCamera(
    hardwareMap: HardwareMap,
    hardwareName: String,
    stream: StreamSettings,
    detection: DetectionSettings,
) : BallCameraBackend {

    private val processor = BallDetectionProcessor(detection)
    private val portal: VisionPortal
    private val worker: CameraControlWorker

    init {
        val webcam = hardwareMap.tryGet(WebcamName::class.java, hardwareName)
            ?: throw HardwareConfigError("Missing Webcam named \"$hardwareName\" in active configuration.")
        portal = VisionPortal.Builder()
            .setCamera(webcam)
            .setCameraResolution(Size(stream.width, stream.height))
            .setStreamFormat(
                if (stream.format == BallStreamFormat.YUY2) VisionPortal.StreamFormat.YUY2 else VisionPortal.StreamFormat.MJPEG,
            )
            .enableLiveView(true)
            .setShowStatsOverlay(true)
            .addProcessor(processor)
            .build()
        worker = CameraControlWorker(VisionPortalControlPort(portal), ExecutorControlScheduler("ball-camera-controls"))
    }

    override val cameraState: String get() = portal.cameraState?.name ?: "UNKNOWN"
    override val libraryReportedFps: Double get() = portal.fps.toDouble()
    override val processorFaults: Long get() = processor.faults
    override val lastProcessorError: String? get() = processor.lastError

    override fun latestFrame(): BallFrameResult? = processor.latestFrame
    override fun lensReport(): LensReport = processor.lensReport
    override fun publishDetectionSettings(settings: DetectionSettings, version: Long) =
        processor.publishSettings(settings, version)

    override fun requestCameraImage(request: CameraImageRequest) = worker.request(request)
    override fun controlStatus(): CameraControlStatus = worker.status

    /**
     * `VisionPortal.close()` can wait on the SDK's camera-state semaphore while
     * the device is still opening, so it runs on its own thread and this call
     * waits at most [timeoutMs] in total. Native buffers are freed only once the
     * device reports closed, so no frame can still be in the processor; on
     * timeout they are left for the process rather than risk a crash.
     */
    override fun close(timeoutMs: Long): Boolean {
        val deadline = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(timeoutMs)
        worker.close()
        val closer = Thread({
            try {
                portal.close()
            } catch (_: Throwable) {
                // Best effort; EasyOpenCV also closes the camera after the OpMode stops.
            }
        }, "ball-camera-close")
        closer.isDaemon = true
        closer.start()
        try {
            closer.join(remainingMs(deadline).coerceAtLeast(1))
        } catch (_: InterruptedException) {
            Thread.currentThread().interrupt()
        }
        if (closer.isAlive || !waitForClosed(deadline)) return false
        processor.releaseBuffers()
        return true
    }

    private fun remainingMs(deadline: Long): Long = TimeUnit.NANOSECONDS.toMillis(deadline - System.nanoTime())

    private fun waitForClosed(deadline: Long): Boolean {
        while (portal.cameraState != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            if (remainingMs(deadline) <= 0) return false
            try {
                Thread.sleep(10)
            } catch (_: InterruptedException) {
                Thread.currentThread().interrupt()
                return false
            }
        }
        return true
    }
}

/** SDK camera controls behind [CameraControlPort]. Called only from the control worker thread. */
private class VisionPortalControlPort(private val portal: VisionPortal) : CameraControlPort {

    override fun isStreaming(): Boolean = portal.cameraState == VisionPortal.CameraState.STREAMING

    private inline fun <reified T : org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl> control(): T? =
        try {
            portal.getCameraControl(T::class.java)
        } catch (_: RuntimeException) {
            null
        }

    override fun probe(): CameraCapabilities {
        val notes = ArrayList<String>()
        val exposure = control<ExposureControl>()
        val gain = control<GainControl>()
        val whiteBalance = control<WhiteBalanceControl>()
        val focus = control<FocusControl>()
        if (exposure == null) notes += "no ExposureControl"
        if (gain == null) notes += "no GainControl"
        if (whiteBalance == null) notes += "no WhiteBalanceControl"
        if (focus == null) notes += "no FocusControl"

        val exposureSupported = exposure?.isExposureSupported == true
        val autoMode = exposure?.let { e ->
            listOf(ExposureControl.Mode.ContinuousAuto, ExposureControl.Mode.Auto, ExposureControl.Mode.AperturePriority)
                .firstOrNull { e.isModeSupported(it) }?.name
        }
        val minGain = gain?.minGain ?: 0
        val maxGain = gain?.maxGain ?: 0
        val minKelvin = whiteBalance?.minWhiteBalanceTemperature ?: 0
        val maxKelvin = whiteBalance?.maxWhiteBalanceTemperature ?: 0
        return CameraCapabilities(
            exposureSupported = exposureSupported,
            manualExposureSupported = exposure?.isModeSupported(ExposureControl.Mode.Manual) == true,
            autoExposureMode = autoMode,
            minExposureMicros = exposure?.getMinExposure(TimeUnit.MICROSECONDS) ?: 0,
            maxExposureMicros = exposure?.getMaxExposure(TimeUnit.MICROSECONDS) ?: 0,
            gainSupported = gain != null && maxGain > minGain,
            minGain = minGain,
            maxGain = maxGain,
            whiteBalanceSupported = whiteBalance != null && maxKelvin > minKelvin,
            minWhiteBalanceKelvin = minKelvin,
            maxWhiteBalanceKelvin = maxKelvin,
            focusLengthSupported = focus?.isFocusLengthSupported == true,
            focusModes = focus?.let { f -> FocusControl.Mode.values().filter { f.isModeSupported(it) }.map { it.name } }
                ?: emptyList(),
            probeNotes = notes,
        )
    }

    override fun setExposureMode(manual: Boolean, autoMode: String?): Boolean {
        val exposure = control<ExposureControl>() ?: return false
        val mode = if (manual) ExposureControl.Mode.Manual else autoMode?.let { ExposureControl.Mode.valueOf(it) } ?: return false
        return exposure.mode == mode || exposure.setMode(mode)
    }

    override fun setExposureMicros(micros: Long): Boolean =
        control<ExposureControl>()?.setExposure(micros, TimeUnit.MICROSECONDS) ?: false

    override fun setGain(gain: Int): Boolean = control<GainControl>()?.setGain(gain) ?: false

    override fun setWhiteBalanceMode(manual: Boolean): Boolean {
        val wb = control<WhiteBalanceControl>() ?: return false
        val mode = if (manual) WhiteBalanceControl.Mode.MANUAL else WhiteBalanceControl.Mode.AUTO
        return wb.mode == mode || wb.setMode(mode)
    }

    override fun setWhiteBalanceKelvin(kelvin: Int): Boolean =
        control<WhiteBalanceControl>()?.setWhiteBalanceTemperature(kelvin) ?: false

    override fun setLiveViewEnabled(enabled: Boolean) {
        if (enabled) portal.resumeLiveView() else portal.stopLiveView()
    }

    override fun readback(): CameraImageReadback {
        val exposure = control<ExposureControl>()
        val gain = control<GainControl>()
        val wb = control<WhiteBalanceControl>()
        return CameraImageReadback(
            exposureMode = exposure?.mode?.name,
            exposureMicros = exposure?.getExposure(TimeUnit.MICROSECONDS)?.takeIf { it > 0 },
            gain = gain?.gain,
            whiteBalanceMode = wb?.mode?.name,
            whiteBalanceKelvin = wb?.whiteBalanceTemperature,
        )
    }
}
