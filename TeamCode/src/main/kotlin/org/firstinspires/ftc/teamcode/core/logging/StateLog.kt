package org.firstinspires.ftc.teamcode.core.logging

/**
 * Per-tick state sink a subsystem writes its log channels into — see
 * [org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase.logState].
 *
 * Channel names are relative; the flight recorder prefixes them with
 * `<subsystem name>/` and lazily creates one WPILOG entry per unique name.
 * Keep a channel's type stable across ticks (the first put fixes it).
 * Implementations must be cheap — this runs on the hot loop every tick.
 */
interface StateLog {
    fun put(channel: String, value: Double)
    fun put(channel: String, value: Long)
    fun put(channel: String, value: Boolean)

    /** Strings are de-duplicated: only written when the value changes. */
    fun put(channel: String, value: String)
}
