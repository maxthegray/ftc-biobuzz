package org.firstinspires.ftc.teamcode.vision.ball

import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

/** What the probe found. Ranges are meaningful only when the matching `*Supported` flag is true. */
data class CameraCapabilities(
    val exposureSupported: Boolean = false,
    val manualExposureSupported: Boolean = false,
    /** First supported of ContinuousAuto, Auto, AperturePriority; null if none. */
    val autoExposureMode: String? = null,
    val minExposureMicros: Long = 0,
    val maxExposureMicros: Long = 0,
    val gainSupported: Boolean = false,
    val minGain: Int = 0,
    val maxGain: Int = 0,
    val whiteBalanceSupported: Boolean = false,
    val minWhiteBalanceKelvin: Int = 0,
    val maxWhiteBalanceKelvin: Int = 0,
    /** Reported for the lab only; no focus setting is offered. */
    val focusLengthSupported: Boolean = false,
    val focusModes: List<String> = emptyList(),
    val probeNotes: List<String> = emptyList(),
)

/** Values read back from the device after writes, or on the periodic refresh. Null = unavailable. */
data class CameraImageReadback(
    val exposureMode: String?,
    val exposureMicros: Long?,
    val gain: Int?,
    val whiteBalanceMode: String?,
    val whiteBalanceKelvin: Int?,
)

enum class CameraControlState { WAITING_FOR_STREAM, READY, CLOSED }

data class CameraControlStatus(
    val state: CameraControlState = CameraControlState.WAITING_FOR_STREAM,
    val capabilities: CameraCapabilities? = null,
    val requested: CameraImageRequest? = null,
    /** The last request whose writes have been attempted. */
    val applied: CameraImageRequest? = null,
    val readback: CameraImageReadback? = null,
    /** Clamps, skipped unsupported controls, and rejected writes from the last apply. */
    val notes: List<String> = emptyList(),
    val deviceWrites: Long = 0,
    val lastError: String? = null,
)

/**
 * The device calls the worker needs. Every method may block on USB control
 * transfers, which is why only the worker thread calls them.
 */
interface CameraControlPort {
    fun isStreaming(): Boolean
    fun probe(): CameraCapabilities
    fun setExposureMode(manual: Boolean, autoMode: String?): Boolean
    fun setExposureMicros(micros: Long): Boolean
    fun setGain(gain: Int): Boolean
    fun setWhiteBalanceMode(manual: Boolean): Boolean
    fun setWhiteBalanceKelvin(kelvin: Int): Boolean
    fun setLiveViewEnabled(enabled: Boolean)
    fun readback(): CameraImageReadback
}

/** Delayed single-thread task execution; replaced by a manual queue in host tests. */
interface ControlScheduler {
    fun schedule(delayMs: Long, task: () -> Unit)
    fun shutdown()
}

class ExecutorControlScheduler(threadName: String) : ControlScheduler {
    private val executor: ScheduledExecutorService = Executors.newSingleThreadScheduledExecutor { runnable ->
        Thread(runnable, threadName).apply { isDaemon = true }
    }

    override fun schedule(delayMs: Long, task: () -> Unit) {
        if (executor.isShutdown) return
        try {
            executor.schedule(task, delayMs, TimeUnit.MILLISECONDS)
        } catch (_: java.util.concurrent.RejectedExecutionException) {
            // Shut down between the check and the submit.
        }
    }

    override fun shutdown() {
        executor.shutdownNow()
    }
}

/**
 * Applies camera image settings off the robot and camera threads.
 *
 * The robot loop calls [request] with every new [CameraImageRequest]; requests
 * made while one is being applied coalesce so only the newest is written. The
 * worker waits for the stream to start, probes capabilities once, clamps to
 * the probed ranges, skips unsupported controls, and writes only the controls
 * whose requested value changed. A failed write is recorded and not retried
 * until the request changes, so a rejecting device cannot cause a write storm.
 * While running it re-reads device values every [refreshIntervalMs] so auto
 * modes show what the camera actually chose.
 */
class CameraControlWorker(
    private val port: CameraControlPort,
    private val scheduler: ControlScheduler,
    private val streamPollMs: Long = 100,
    private val refreshIntervalMs: Long = 1000,
) {
    private val desired = AtomicReference<CameraImageRequest?>(null)
    private val drainQueued = AtomicBoolean(false)
    private val refreshQueued = AtomicBoolean(false)
    private val closed = AtomicBoolean(false)
    private val statusRef = AtomicReference(CameraControlStatus())

    private var capabilities: CameraCapabilities? = null
    private var lastAttempted: CameraImageRequest? = null
    private var writes = 0L

    val status: CameraControlStatus get() = statusRef.get()

    fun request(request: CameraImageRequest) {
        if (closed.get()) return
        desired.set(request)
        statusRef.updateAndGet { it.copy(requested = request) }
        queueDrain(0)
    }

    fun close() {
        if (!closed.compareAndSet(false, true)) return
        scheduler.shutdown()
        statusRef.updateAndGet { it.copy(state = CameraControlState.CLOSED) }
    }

    private fun queueDrain(delayMs: Long) {
        if (closed.get()) return
        if (drainQueued.compareAndSet(false, true)) scheduler.schedule(delayMs) { drain() }
    }

    private fun drain() {
        drainQueued.set(false)
        if (closed.get()) return
        try {
            if (!port.isStreaming()) {
                statusRef.updateAndGet { it.copy(state = CameraControlState.WAITING_FOR_STREAM) }
                queueDrain(streamPollMs)
                return
            }
            val caps = capabilities ?: port.probe().also {
                capabilities = it
                statusRef.updateAndGet { s -> s.copy(state = CameraControlState.READY, capabilities = it) }
                queueRefresh()
            }
            val target = desired.get() ?: return
            if (target == lastAttempted) return
            val notes = apply(target, lastAttempted, caps)
            lastAttempted = target
            statusRef.updateAndGet {
                it.copy(
                    state = CameraControlState.READY,
                    applied = target,
                    notes = notes,
                    deviceWrites = writes,
                    readback = safeReadback() ?: it.readback,
                )
            }
        } catch (t: Throwable) {
            lastAttempted = desired.get()
            statusRef.updateAndGet { it.copy(lastError = "${t.javaClass.simpleName}: ${t.message}", deviceWrites = writes) }
        }
        if (desired.get() != lastAttempted) queueDrain(0)
    }

    private fun queueRefresh() {
        if (closed.get() || refreshIntervalMs <= 0) return
        if (refreshQueued.compareAndSet(false, true)) {
            scheduler.schedule(refreshIntervalMs) {
                refreshQueued.set(false)
                if (closed.get()) return@schedule
                safeReadback()?.let { rb -> statusRef.updateAndGet { it.copy(readback = rb) } }
                queueRefresh()
            }
        }
    }

    private fun safeReadback(): CameraImageReadback? = try {
        port.readback()
    } catch (t: Throwable) {
        statusRef.updateAndGet { it.copy(lastError = "readback ${t.javaClass.simpleName}: ${t.message}") }
        null
    }

    private fun apply(
        target: CameraImageRequest,
        previous: CameraImageRequest?,
        caps: CameraCapabilities,
    ): List<String> {
        val notes = ArrayList<String>()

        if (previous == null || target.liveViewEnabled != previous.liveViewEnabled) {
            port.setLiveViewEnabled(target.liveViewEnabled)
        }

        val exposureModeChanged = previous == null || target.exposureManual != previous.exposureManual
        if (!caps.exposureSupported) {
            notes += "exposure control unsupported"
        } else {
            if (exposureModeChanged) {
                if (target.exposureManual && !caps.manualExposureSupported) {
                    notes += "manual exposure mode unsupported"
                } else if (!target.exposureManual && caps.autoExposureMode == null) {
                    notes += "no automatic exposure mode supported"
                } else {
                    write("exposure mode", notes) { port.setExposureMode(target.exposureManual, caps.autoExposureMode) }
                }
            }
            if (target.exposureManual && caps.manualExposureSupported &&
                (exposureModeChanged || target.exposureMicros != previous?.exposureMicros)
            ) {
                val micros = clamp(target.exposureMicros, caps.minExposureMicros, caps.maxExposureMicros)
                if (micros != target.exposureMicros) notes += "exposure clamped to $micros µs"
                write("exposure", notes) { port.setExposureMicros(micros) }
            }
        }

        if (target.gain >= 0) {
            if (!caps.gainSupported) {
                notes += "gain unsupported by probe; ignored"
            } else if (!target.exposureManual) {
                notes += "gain waits for manual exposure"
            } else if (exposureModeChanged || target.gain != previous?.gain) {
                val gain = clamp(target.gain.toLong(), caps.minGain.toLong(), caps.maxGain.toLong()).toInt()
                if (gain != target.gain) notes += "gain clamped to $gain"
                write("gain", notes) { port.setGain(gain) }
            }
        }

        if (!caps.whiteBalanceSupported) {
            notes += "white balance unsupported"
        } else {
            val wbModeChanged = previous == null || target.whiteBalanceManual != previous.whiteBalanceManual
            if (wbModeChanged) write("white balance mode", notes) { port.setWhiteBalanceMode(target.whiteBalanceManual) }
            if (target.whiteBalanceManual && (wbModeChanged || target.whiteBalanceKelvin != previous?.whiteBalanceKelvin)) {
                val kelvin = clamp(
                    target.whiteBalanceKelvin.toLong(),
                    caps.minWhiteBalanceKelvin.toLong(),
                    caps.maxWhiteBalanceKelvin.toLong(),
                ).toInt()
                if (kelvin != target.whiteBalanceKelvin) notes += "white balance clamped to $kelvin K"
                write("white balance", notes) { port.setWhiteBalanceKelvin(kelvin) }
            }
        }
        return notes
    }

    private inline fun write(what: String, notes: MutableList<String>, call: () -> Boolean) {
        writes++
        if (!call()) notes += "$what write rejected by device"
    }

    private fun clamp(value: Long, min: Long, max: Long): Long =
        if (max > min) value.coerceIn(min, max) else value
}
