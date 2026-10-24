package org.firstinspires.ftc.teamcode.core.logging

import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.junit.After
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
        internal val hidden = 13.0
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
    fun mechanismChannelsMatchTheirFormerExplicitLoggers() {
        val intake = Capture().also { IntakeSubsystem().logState(it) }
        assertEquals(mapOf("power" to 0.0), intake.values)
        val transfer = Capture().also { TransferSubsystem().logState(it) }
        assertEquals(
            mapOf("motorPower" to 0.0, "blockerOpen" to false, "blockerSettled" to false, "blockerPosition" to Double.NaN),
            transfer.values,
        )
        val turret = Capture().also { TurretSubsystem().logState(it) }
        val shooter = Capture().also { ShooterSubsystem().logState(it) }
        assertEquals(mapOf("configured" to false), turret.values)
        assertEquals(mapOf("configured" to false), shooter.values)
    }

    @After
    fun resetEvents() = Events.reset()

    @Test
    fun onlyDeclaredPublicReadOnlyScalarsAreLoggedAndABrokenGetterIsDroppedOnce() {
        val events = mutableListOf<String>()
        Events.sink = events::add
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
        assertEquals(listOf("SUBSYSTEM LOGGING DISABLED: Test/broken: IllegalStateException: unreadable"), events)

        // A new instance (the next op-mode) tries the getter again.
        State().logState(Capture())
        assertEquals(2, events.size)
    }
}
