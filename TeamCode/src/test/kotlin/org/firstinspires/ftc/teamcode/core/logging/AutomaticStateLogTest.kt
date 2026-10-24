package org.firstinspires.ftc.teamcode.core.logging

import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.junit.Assert.assertEquals
import org.junit.Test

class AutomaticStateLogTest {
    enum class Mode { READY }

    open class Parent : SubsystemBase("Test") {
        val inherited = 9.0
    }

    class State : Parent() {
        val double = 1.5
        val float = 2.5f
        val int = 3
        val long = 4L
        val boolean = true
        val string = "hello"
        val mode = Mode.READY
        val nullable: Double? = 6.0
        val missing: String? = null
        val missingEnum: Mode? = null
        val broken: Double get() = error("unreadable")
        val zAfterBroken = 7.0
        private val secret = 8.0
        var writable = 10.0
        var privateSetter = 11.0
            private set
        val array = doubleArrayOf(1.0)
        fun getWithArgument(value: Int): Int = value
        fun ordinary(): Double = 12.0
        fun isReady(): Boolean = true
    }

    private class Capture : StateLog {
        val values = linkedMapOf<String, Any>()
        override fun put(channel: String, value: Double) { values[channel] = value }
        override fun put(channel: String, value: Long) { values[channel] = value }
        override fun put(channel: String, value: Boolean) { values[channel] = value }
        override fun put(channel: String, value: String) { values[channel] = value }
        override fun put(channel: String, value: DoubleArray) { values[channel] = value }
    }

    @Test
    fun onlyDeclaredPublicReadOnlyScalarsAreLoggedAndBrokenGettersAreSkipped() {
        val state = State()
        repeat(2) {
            val log = Capture()
            state.logState(log)
            assertEquals(
                mapOf(
                    "double" to 1.5, "float" to 2.5, "int" to 3L, "long" to 4L,
                    "boolean" to true, "string" to "hello", "mode" to "READY",
                    "nullable" to 6.0, "zAfterBroken" to 7.0, "privateSetter" to 11.0,
                    "ready" to true,
                ),
                log.values,
            )
        }
    }
}
