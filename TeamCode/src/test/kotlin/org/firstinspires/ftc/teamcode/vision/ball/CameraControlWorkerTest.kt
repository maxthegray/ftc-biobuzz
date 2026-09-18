package org.firstinspires.ftc.teamcode.vision.ball

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class CameraControlWorkerTest {

    private val port = FakePort()
    private val scheduler = ManualScheduler()
    private val worker = CameraControlWorker(port, scheduler, streamPollMs = 100, refreshIntervalMs = 1000)

    private val auto = CameraImageRequest(
        exposureManual = false, exposureMicros = 5000, gain = -1,
        whiteBalanceManual = false, whiteBalanceKelvin = 4600, liveViewEnabled = true,
    )

    @Test
    fun requestReturnsImmediatelyAndDoesNothingOnTheCallingThread() {
        worker.request(auto)
        assertTrue(port.calls.isEmpty())
        assertEquals(auto, worker.status.requested)
        assertNull(worker.status.applied)
    }

    @Test
    fun waitsForStreamingThenProbesOnceBeforeWriting() {
        port.streaming = false
        worker.request(auto)
        scheduler.runDue()
        assertEquals(CameraControlState.WAITING_FOR_STREAM, worker.status.state)
        assertEquals(listOf("isStreaming"), port.calls)

        port.streaming = true
        scheduler.advance(100)
        scheduler.runDue()
        assertEquals(1, port.calls.count { it == "probe" })
        assertEquals(CameraControlState.READY, worker.status.state)
        assertEquals(auto, worker.status.applied)
        assertTrue("exposure mode" in port.calls.joinToString())

        worker.request(auto.copy(exposureManual = true))
        scheduler.runDue()
        assertEquals(1, port.calls.count { it == "probe" })
    }

    @Test
    fun rapidRequestsCoalesceIntoOneApplyOfTheNewest() {
        worker.request(auto.copy(exposureManual = true, exposureMicros = 1000))
        worker.request(auto.copy(exposureManual = true, exposureMicros = 2000))
        worker.request(auto.copy(exposureManual = true, exposureMicros = 3000))
        scheduler.runDue()

        assertEquals(listOf(3000L), port.exposureWrites)
        assertEquals(3000L, worker.status.applied!!.exposureMicros)
    }

    @Test
    fun unchangedRequestsCauseNoDeviceWrites() {
        worker.request(auto)
        scheduler.runDue()
        val writes = worker.status.deviceWrites
        repeat(5) {
            worker.request(auto)
            scheduler.runDue()
        }
        assertEquals(writes, worker.status.deviceWrites)
    }

    @Test
    fun onlyChangedControlsAreWritten() {
        val manual = auto.copy(exposureManual = true, exposureMicros = 4000, whiteBalanceManual = true, whiteBalanceKelvin = 5000)
        worker.request(manual)
        scheduler.runDue()
        port.calls.clear()

        worker.request(manual.copy(whiteBalanceKelvin = 5200))
        scheduler.runDue()
        assertEquals(listOf("wbKelvin 5200"), port.calls.filter { it != "isStreaming" && it != "readback" })
    }

    @Test
    fun valuesAreClampedToProbedRangesAndReported() {
        worker.request(auto.copy(exposureManual = true, exposureMicros = 99_000, whiteBalanceManual = true, whiteBalanceKelvin = 100))
        scheduler.runDue()

        assertEquals(listOf(8000L), port.exposureWrites)
        assertTrue("wbKelvin 2800" in port.calls)
        assertTrue(worker.status.notes.any { "exposure clamped to 8000" in it })
        assertTrue(worker.status.notes.any { "white balance clamped to 2800" in it })
    }

    @Test
    fun gainIsSkippedWhenProbeFindsNoRangeAndWaitsForManualExposure() {
        port.capabilities = port.capabilities.copy(gainSupported = false)
        worker.request(auto.copy(exposureManual = true, gain = 50))
        scheduler.runDue()
        assertTrue(port.calls.none { it.startsWith("gain") })
        assertTrue(worker.status.notes.any { "gain unsupported" in it })

        val supported = FakePort()
        val otherScheduler = ManualScheduler()
        val autoExposureWorker = CameraControlWorker(supported, otherScheduler, refreshIntervalMs = 0)
        autoExposureWorker.request(auto.copy(gain = 50))
        otherScheduler.runDue()
        assertTrue(supported.calls.none { it.startsWith("gain") })
        assertTrue(autoExposureWorker.status.notes.any { "manual exposure" in it })
    }

    @Test
    fun rejectedWriteIsRecordedAndNotRetriedUntilTheRequestChanges() {
        port.acceptWrites = false
        worker.request(auto.copy(exposureManual = true, exposureMicros = 3000))
        scheduler.runDue()
        assertTrue(worker.status.notes.any { "rejected" in it })
        val attempts = port.exposureWrites.size

        repeat(3) { scheduler.advance(1000); scheduler.runDue() }
        assertEquals(attempts, port.exposureWrites.size)

        worker.request(auto.copy(exposureManual = true, exposureMicros = 3500))
        scheduler.runDue()
        assertEquals(attempts + 1, port.exposureWrites.size)
    }

    @Test
    fun deviceExceptionIsContainedAndReported() {
        port.throwOnWrite = true
        worker.request(auto.copy(exposureManual = true))
        scheduler.runDue()
        assertTrue(worker.status.lastError!!.contains("IllegalStateException"))
    }

    @Test
    fun periodicReadbackShowsWhatAutoModesChose() {
        worker.request(auto)
        scheduler.runDue()
        port.readbackExposure = 7777
        scheduler.advance(1000)
        scheduler.runDue()
        assertEquals(7777L, worker.status.readback!!.exposureMicros)
    }

    @Test
    fun closeStopsFurtherWork() {
        worker.close()
        worker.request(auto)
        scheduler.runDue()
        assertTrue(port.calls.isEmpty())
        assertEquals(CameraControlState.CLOSED, worker.status.state)
        assertTrue(scheduler.shutdown)
    }

    private class ManualScheduler : ControlScheduler {
        private data class Task(val dueMs: Long, val run: () -> Unit)
        private val tasks = ArrayList<Task>()
        private var nowMs = 0L
        var shutdown = false

        override fun schedule(delayMs: Long, task: () -> Unit) {
            if (!shutdown) tasks += Task(nowMs + delayMs, task)
        }

        override fun shutdown() {
            shutdown = true
            tasks.clear()
        }

        fun advance(ms: Long) {
            nowMs += ms
        }

        fun runDue() {
            while (true) {
                val next = tasks.filter { it.dueMs <= nowMs }.minByOrNull { it.dueMs } ?: return
                tasks.remove(next)
                next.run()
            }
        }
    }

    private class FakePort : CameraControlPort {
        var streaming = true
        var acceptWrites = true
        var throwOnWrite = false
        var readbackExposure = 5000L
        val calls = ArrayList<String>()
        val exposureWrites = ArrayList<Long>()
        var capabilities = CameraCapabilities(
            exposureSupported = true, manualExposureSupported = true, autoExposureMode = "AperturePriority",
            minExposureMicros = 100, maxExposureMicros = 8000,
            gainSupported = true, minGain = 0, maxGain = 100,
            whiteBalanceSupported = true, minWhiteBalanceKelvin = 2800, maxWhiteBalanceKelvin = 6500,
        )

        override fun isStreaming(): Boolean { calls += "isStreaming"; return streaming }
        override fun probe(): CameraCapabilities { calls += "probe"; return capabilities }
        override fun setExposureMode(manual: Boolean, autoMode: String?): Boolean = write("exposure mode $manual")
        override fun setExposureMicros(micros: Long): Boolean { exposureWrites += micros; return write("exposure $micros") }
        override fun setGain(gain: Int): Boolean = write("gain $gain")
        override fun setWhiteBalanceMode(manual: Boolean): Boolean = write("wbMode $manual")
        override fun setWhiteBalanceKelvin(kelvin: Int): Boolean = write("wbKelvin $kelvin")
        override fun setLiveViewEnabled(enabled: Boolean) { calls += "liveView $enabled" }
        override fun readback(): CameraImageReadback {
            calls += "readback"
            return CameraImageReadback("Manual", readbackExposure, 10, "AUTO", 4600)
        }

        private fun write(call: String): Boolean {
            calls += call
            if (throwOnWrite) throw IllegalStateException("usb")
            return acceptWrites
        }
    }
}
