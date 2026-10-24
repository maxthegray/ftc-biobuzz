package org.firstinspires.ftc.teamcode.core.logging

/** Timestamped flight-log events, scoped to the current Robot like CommandHistory. */
object Events {
    internal var sink: ((String) -> Unit)? = null

    fun record(message: String) {
        sink?.invoke(message)
    }

    internal fun reset() {
        sink = null
    }
}
