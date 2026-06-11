package org.firstinspires.ftc.teamcode.core.runtime

import java.io.File
import org.firstinspires.ftc.teamcode.core.pathing.PedroAutoRunner
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.firstinspires.ftc.teamcode.core.util.GamepadEx
import org.firstinspires.ftc.teamcode.core.util.TelemetryBag

/**
 * Init-loop autonomous menu controlled by dpad.
 *
 * Up/down moves between fields; left/right changes the selected value. Press
 * A to lock the choice before start. The selected alliance is written to
 * [robot] each update so path builders and [OpModeBase.alliance] share one
 * source of truth during init.
 *
 * The last locked selection is persisted to disk and restored (unlocked) on
 * the next init — re-initializing right before a match doesn't mean
 * re-selecting the routine under pressure. The restore is only a default;
 * it still has to be locked with A.
 */
class AutonSelector(
    private val robot: Robot,
    private val telemetryBag: TelemetryBag,
) {
    private data class Routine(val name: String, val build: () -> PedroAutoRunner)
    private enum class Field { ALLIANCE, ROUTINE, DELAY }

    private val routines = mutableListOf<Routine>()
    private var field = Field.ALLIANCE
    private var alliance = Alliance.RED
    private var routineIndex = 0
    private var delaySec = 0
    private var lockedSelection: Selection? = null

    private data class Selection(
        val alliance: Alliance,
        val routineIndex: Int,
        val delaySec: Int,
    )

    init {
        restoreFromDisk()
    }

    fun register(name: String, build: () -> PedroAutoRunner): AutonSelector {
        routines += Routine(name, build)
        return this
    }

    fun update(driver: GamepadEx) {
        if (lockedSelection == null) {
            if (driver.dpadUpPressed) field = Field.entries[(field.ordinal + Field.entries.size - 1) % Field.entries.size]
            if (driver.dpadDownPressed) field = Field.entries[(field.ordinal + 1) % Field.entries.size]
            if (driver.dpadLeftPressed) step(-1)
            if (driver.dpadRightPressed) step(1)
            if (driver.aPressed) {
                lockedSelection = Selection(alliance, clampedRoutineIndex, delaySec)
                persistToDisk()
            }
        }

        robot.alliance = selectedAlliance
        render()
    }

    fun selectedRunner(): PedroAutoRunner? = routines.getOrNull(selection.routineIndex)?.build?.invoke()

    val isLocked: Boolean get() = lockedSelection != null
    val selectedAlliance: Alliance get() = selection.alliance
    val selectedRoutineName: String get() = routines.getOrNull(selection.routineIndex)?.name ?: "-"
    val startDelaySec: Int get() = selection.delaySec

    private val selection: Selection
        get() = lockedSelection ?: Selection(alliance, clampedRoutineIndex, delaySec)

    /** A restored index may point past the routines registered this run. */
    private val clampedRoutineIndex: Int
        get() = if (routines.isEmpty()) 0 else routineIndex.coerceIn(0, routines.size - 1)

    private fun step(delta: Int) {
        when (field) {
            Field.ALLIANCE -> alliance = if (alliance == Alliance.RED) Alliance.BLUE else Alliance.RED
            Field.ROUTINE -> if (routines.isNotEmpty()) {
                routineIndex = (routineIndex + delta + routines.size) % routines.size
            }
            Field.DELAY -> delaySec = (delaySec + delta).coerceIn(0, 10)
        }
    }

    private fun render() {
        telemetryBag.section("Auton") {
            put("status", if (isLocked) "LOCKED" else "EDIT")
            put("field", field.name)
            put("alliance", selectedAlliance)
            put("routine", selectedRoutineName)
            put("delay s", startDelaySec)
        }
    }

    private fun persistToDisk() {
        val file = storageFile ?: return
        try {
            file.parentFile?.mkdirs()
            file.writeText("$MAGIC ${alliance.name} $clampedRoutineIndex $delaySec")
        } catch (_: Throwable) {
            // Best-effort: losing the default costs one re-selection.
        }
    }

    private fun restoreFromDisk() {
        val file = storageFile ?: return
        try {
            if (!file.exists()) return
            val parts = file.readText().trim().split(" ")
            if (parts.size != 4 || parts[0] != MAGIC) return
            alliance = Alliance.entries.firstOrNull { it.name == parts[1] } ?: return
            routineIndex = (parts[2].toIntOrNull() ?: return).coerceAtLeast(0)
            delaySec = (parts[3].toIntOrNull() ?: return).coerceIn(0, 10)
        } catch (_: Throwable) {
            // Unreadable file == no default.
        }
    }

    companion object {
        private const val MAGIC = "auton-v1"

        /** Overridable for host tests; null disables persistence. */
        internal var storageFile: File? = File("/sdcard/FIRST/auton-selection.txt")
    }
}
