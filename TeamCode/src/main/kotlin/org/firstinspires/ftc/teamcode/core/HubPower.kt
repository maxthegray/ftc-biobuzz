package org.firstinspires.ftc.teamcode.core

import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit

/**
 * Each hub's input current and voltage: everything the hub powers, so new
 * mechanisms are counted without registering anything. Both are real Lynx
 * transactions, so one hub is read every [SAMPLE_INTERVAL_NS], round-robin,
 * current and voltage back to back so they describe the same moment.
 */
class HubPower(private val hubs: List<Hub>, private val clock: Clock) {

    interface Hub {
        val label: String
        fun amps(): Double
        fun volts(): Double
    }

    class Reading(val label: String) {
        var amps = Double.NaN
            internal set
        var volts = Double.NaN
            internal set

        /** A reading the flight recorder hasn't written yet. */
        var unlogged = false
    }

    val readings: List<Reading> = hubs.map { Reading(it.label) }

    /** The first hub's (the Control Hub's) latest input voltage, NaN until read. */
    val batteryVolts: Double get() = readings.firstOrNull()?.volts ?: Double.NaN

    private var next = 0
    private var lastSampleNs = Long.MIN_VALUE

    fun sample() {
        if (hubs.isEmpty()) return
        val now = clock.nanos()
        if (lastSampleNs != Long.MIN_VALUE && now - lastSampleNs < SAMPLE_INTERVAL_NS) return
        lastSampleNs = now
        val index = next
        next = (next + 1) % hubs.size
        try {
            val amps = hubs[index].amps()
            val volts = hubs[index].volts()
            readings[index].apply {
                this.amps = amps
                this.volts = volts
                unlogged = true
            }
        } catch (_: Throwable) {
            // Logging must never stop the robot; the channel just goes quiet.
        }
    }

    companion object {
        const val SAMPLE_INTERVAL_NS = 50_000_000L

        /** The Control Hub first, labelled `control`; Expansion Hubs as `expansion<address>`. */
        fun of(modules: List<LynxModule>, clock: Clock): HubPower =
            HubPower(modules.sortedByDescending { it.isParent }.map(::LynxHub), clock)
    }

    private class LynxHub(private val module: LynxModule) : Hub {
        override val label = if (module.isParent) "control" else "expansion${module.moduleAddress}"
        override fun amps() = module.getCurrent(CurrentUnit.AMPS)
        override fun volts() = module.getInputVoltage(VoltageUnit.VOLTS)
    }
}
