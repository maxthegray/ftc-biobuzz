package org.firstinspires.ftc.teamcode.core

/** A one-shot cue, with both times measured from op-mode start. */
data class RumbleAlert(val elapsedNanos: Long, val blips: Int, val expiresNanos: Long)

/** One instance per run. A delayed tick emits only the latest unexpired cue. */
internal class RumbleAlerts(alerts: List<RumbleAlert>) {
    private val alerts = alerts.sortedBy { it.elapsedNanos }
    private var next = 0

    fun poll(elapsedNanos: Long): RumbleAlert? {
        var latest: RumbleAlert? = null
        while (next < alerts.size && elapsedNanos >= alerts[next].elapsedNanos) {
            latest = alerts[next++]
        }
        return latest?.takeIf { elapsedNanos < it.expiresNanos }
    }
}
