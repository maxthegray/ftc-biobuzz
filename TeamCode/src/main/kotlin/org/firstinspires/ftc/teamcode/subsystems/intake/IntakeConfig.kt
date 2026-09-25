package org.firstinspires.ftc.teamcode.subsystems.intake

import com.bylazar.configurables.annotations.Configurable

/**
 * Intake powers, persisted under `intake`. Placeholders until the intake is
 * built: tune in Panels.
 */
@Configurable
object IntakeConfig {

    private const val DEFAULT_COLLECT_POWER = 1.0
    private const val DEFAULT_EJECT_POWER = -0.6

    @JvmField var collectPower: Double = DEFAULT_COLLECT_POWER
    @JvmField var ejectPower: Double = DEFAULT_EJECT_POWER

    fun resetDefaults() {
        collectPower = DEFAULT_COLLECT_POWER
        ejectPower = DEFAULT_EJECT_POWER
    }

    internal val safeCollectPower: Double get() = power(collectPower, DEFAULT_COLLECT_POWER)
    internal val safeEjectPower: Double get() = power(ejectPower, DEFAULT_EJECT_POWER)

    private fun power(value: Double, default: Double) = if (value.isFinite()) value.coerceIn(-1.0, 1.0) else default
}
