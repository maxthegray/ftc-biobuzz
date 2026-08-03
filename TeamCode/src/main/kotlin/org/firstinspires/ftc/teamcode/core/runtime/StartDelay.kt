package org.firstinspires.ftc.teamcode.core.runtime

import org.firstinspires.ftc.teamcode.core.util.GamepadEx
import org.firstinspires.ftc.teamcode.core.util.TelemetryBag

/**
 * Init-loop start delay on dpad left/right, 0..[maxSec] seconds.
 *
 * This is the only auton choice that can't be a separate op-mode: alliance,
 * side, and routine are all known at build time and belong in the Driver
 * Station dropdown, where the selected name is displayed in large text. The
 * delay is picked in the alliance meeting minutes before a match, to stay out
 * of a partner routine's way, so it has to be adjustable at init.
 *
 * Deliberately holds no lock and persists nothing — the displayed value is
 * always the one that runs.
 */
class StartDelay(
    private val telemetryBag: TelemetryBag,
    private val maxSec: Int = 10,
) {
    var seconds: Int = 0
        private set

    val millis: Long get() = seconds * 1000L

    fun update(driver: GamepadEx) {
        if (driver.dpadRightPressed) seconds = (seconds + 1).coerceAtMost(maxSec)
        if (driver.dpadLeftPressed) seconds = (seconds - 1).coerceAtLeast(0)
        telemetryBag.section("Auton") {
            put("start delay s", seconds)
        }
    }
}
