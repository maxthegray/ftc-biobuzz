package org.firstinspires.ftc.teamcode.core.hardware

import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.max

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
    private val periodNs = (1_000_000_000.0 / targetHz).toLong().coerceAtLeast(250_000L)
    private val running = AtomicBoolean(false)
    private val iterations = AtomicLong(0)
    private val lastPollNs = AtomicLong(0)

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

    fun stop() {
        if (!running.compareAndSet(true, false)) return
        thread?.interrupt()
        thread = null
    }

    val pollCount: Long get() = iterations.get()
    val lastPollDurationNs: Long get() = lastPollNs.get()
    val isRunning: Boolean get() = running.get()

    private fun runLoop() {
        while (running.get() && !Thread.currentThread().isInterrupted) {
            val startNs = System.nanoTime()
            try {
                ref.publish(poll())
            } catch (t: Throwable) {
                // Swallow and keep polling — a transient I²C glitch should
                // not kill the bus thread. The publish is skipped so the
                // main loop keeps the last good value.
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
