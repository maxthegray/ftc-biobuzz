package org.firstinspires.ftc.teamcode.core.hardware

import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicReference
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * A measured-need escape hatch for polling one I2C-bound device at a fixed
 * target rate and publishing the result via a [Ref]. The default architecture
 * keeps Pinpoint direct and reads the SRSHub inline; read `SENSORS.md` before
 * using this class. Backgrounding a device on the Control Hub's Lynx link can
 * delay motor writes even when it shortens the apparent main-loop time.
 *
 * Usage:
 *
 * ```kotlin
 * val bus = I2CBusThread("independent-aux-sensor", 50.0) {
 *     val reading = independentAdapter.read()
 *     AuxSnapshot(reading.distanceMm, reading.status)
 * }
 * bus.start()
 * // somewhere in your loop
 * val pose = bus.ref.get()
 * // at stop()
 * bus.stop()
 * ```
 *
 * Contract:
 *  - The [poll] block is called from a background thread. It must be
 *    thread-safe with respect to its own hardware handle; FTC Lynx I²C
 *    devices are — each Lynx bus has its own internal lock.
 *  - The published value must be immutable or independently cloned each
 *    poll. Do NOT mutate a shared object and re-publish it.
 *  - The main loop only ever reads via [Ref.get]; it never blocks.
 */
class I2CBusThread<T : Any>(
    private val name: String,
    targetHz: Double,
    initial: T? = null,
    private val clock: Clock = Clock.SYSTEM,
    private val poll: () -> T,
) {
    init {
        require(targetHz > 0.0) { "targetHz must be positive" }
    }

    private val periodNs = (1_000_000_000.0 / targetHz).toLong().coerceAtLeast(250_000L)
    private val running = AtomicBoolean(false)
    private val iterations = AtomicLong(0)
    private val failures = AtomicLong(0)
    private val lastPollNs = AtomicLong(0)
    private val lastSuccessNs = AtomicLong(0)
    private val lastErrorRef = AtomicReference<Throwable?>(null)

    val ref = Ref<T>(initial)

    private var thread: Thread? = null

    fun start() {
        if (!running.compareAndSet(false, true)) return
        thread = Thread({ runLoop() }, "i2c-$name").apply {
            isDaemon = true
            priority = Thread.NORM_PRIORITY + 1
            start()
        }
    }

    fun stop(joinTimeoutMs: Long = 100L) {
        if (!running.compareAndSet(true, false)) return
        val t = thread
        t?.interrupt()
        if (t != null && t !== Thread.currentThread()) {
            try {
                t.join(joinTimeoutMs.coerceAtLeast(0L))
            } catch (_: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }
        if (thread === t && (t == null || !t.isAlive)) thread = null
    }

    val pollCount: Long get() = iterations.get()
    val failureCount: Long get() = failures.get()
    val lastPollDurationNs: Long get() = lastPollNs.get()
    val lastSuccessfulPollNs: Long get() = lastSuccessNs.get()
    val lastError: Throwable? get() = lastErrorRef.get()
    val isRunning: Boolean get() = running.get()

    private fun runLoop() {
        while (running.get() && !Thread.currentThread().isInterrupted) {
            val startNs = clock.nanos()
            try {
                ref.publish(poll())
                lastSuccessNs.set(clock.nanos())
                lastErrorRef.set(null)
            } catch (t: Throwable) {
                // Keep polling after a transient I2C glitch, but expose the
                // failure so telemetry can show a stale or unhealthy sensor.
                failures.incrementAndGet()
                lastErrorRef.set(t)
                Thread.yield()
            }
            val elapsed = clock.nanos() - startNs
            lastPollNs.set(elapsed)
            iterations.incrementAndGet()
            val sleepNs = periodNs - elapsed
            if (sleepNs > 0) {
                try {
                    Thread.sleep(sleepNs / 1_000_000L, (sleepNs % 1_000_000L).toInt())
                } catch (_: InterruptedException) {
                    Thread.currentThread().interrupt()
                    break
                }
            }
        }
    }

    /**
     * Volatile single-slot reference. Lock-free because there is exactly one
     * writer (the bus thread) and N readers (main loop), and publishing a
     * reference is atomic on the JVM.
     */
    class Ref<T : Any>(initial: T? = null) {
        @Volatile private var value: T? = initial
        fun get(): T? = value
        fun publish(v: T) { value = v }
        fun has(): Boolean = value != null
    }
}
