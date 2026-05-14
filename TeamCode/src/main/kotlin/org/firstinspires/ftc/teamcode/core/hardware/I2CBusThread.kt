package org.firstinspires.ftc.teamcode.core.hardware

import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicReference

/**
 * A single background thread that polls one I2C-bound device at a fixed
 * target rate and publishes the result via a [Ref]. Designed for things like
 * the GoBilda Pinpoint driver, where each read takes ~2 ms of blocking I²C
 * traffic and would otherwise tank the main-loop frequency.
 *
 * Usage:
 *
 * ```kotlin
 * val bus = I2CBusThread("pinpoint", 200.0) {
 *     pinpoint.update()
 *     pinpoint.position
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
            val startNs = System.nanoTime()
            try {
                ref.publish(poll())
                lastSuccessNs.set(System.nanoTime())
                lastErrorRef.set(null)
            } catch (t: Throwable) {
                // Keep polling after a transient I2C glitch, but expose the
                // failure so telemetry can show a stale or unhealthy sensor.
                failures.incrementAndGet()
                lastErrorRef.set(t)
                Thread.yield()
            }
            val elapsed = System.nanoTime() - startNs
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
