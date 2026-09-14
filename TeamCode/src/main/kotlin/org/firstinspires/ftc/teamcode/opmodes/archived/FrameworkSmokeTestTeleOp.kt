package org.firstinspires.ftc.teamcode.opmodes.archived

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.util.Clock
import org.firstinspires.ftc.teamcode.core.util.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.util.monotonicWaitMs

/** Archived chassis-free runtime check; re-enable for framework or Control Hub bring-up. */
@Disabled
@TeleOp(name = "Framework Smoke Test", group = "Diagnostics")
class FrameworkSmokeTestTeleOp : OpModeBase() {

    private lateinit var smoke: FrameworkSmokeSubsystem

    override val requiredDevices: List<Preflight.Requirement> get() = emptyList()
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        smoke = robot.register(FrameworkSmokeSubsystem(robot.clock, robot::recordEvent))
        smoke.defaultCommand = smoke.idleCommand()

        driver.button(Button.A).onTrue(instant(smoke::recordMarker))
        driver.button(Button.B).onTrue(smoke.timedWorkCommand())
        driver.button(Button.X).onTrue(smoke.overrideCommand())
        driver.button(Button.Y).onTrue(smoke.faultCommand())
        driver.button(Button.LEFT_BUMPER).whileTrue(smoke.heldCommand())
        driver.button(Button.DPAD_UP).onTrue(
            sequential(
                instant { robot.recordEvent("smoke sequence start") },
                monotonicWaitMs(500.0, robot.clock),
                instant(smoke::completeSequence),
            ),
        )
    }

    override fun onInitLoop() = emitTelemetry()

    override fun onLoop() {
        emitTelemetry()
        // With no hardware I/O this otherwise runs near 2 kHz and produces
        // competition-length logs around 100 MB. Pace only this diagnostic.
        sleep(SMOKE_LOOP_DELAY_MS)
    }

    private fun emitTelemetry() {
        telemetryBag.section("Framework Smoke Test") {
            put("SAFETY", "NO CONFIGURED DEVICES OR OUTPUTS")
            put("target loop", "~50 Hz")
            put("phase", if (opModeInInit()) "INIT" else "RUNNING")
            put("state", smoke.state)
            put("periodic ticks", smoke.periodicTicks)
            put("write ticks", smoke.writeTicks)
            put("default ticks", smoke.defaultTicks)
            put("work ticks", smoke.workTicks)
            put("last end", smoke.lastEnd)
            put("markers", smoke.markerCount)
            put("sequences", smoke.sequenceCount)
            put("fault safes", smoke.faultSafeCount)
            put("config input exponent", DriveConfig.inputExponent)
        }
        telemetryBag.section("Controls") {
            put("A", "record WPILOG marker")
            put("B", "run 2-second command")
            put("X", "preempt active subsystem command")
            put("Y", "intentional command fault (aborts all commands)")
            put("hold LB", "run; release to interrupt")
            put("dpad up", "instant + wait + instant sequence")
        }
        telemetryBag.section("Gamepad") {
            put("left stick", "%.2f, %.2f".format(driver.leftStickX, driver.leftStickY))
            put("right stick", "%.2f, %.2f".format(driver.rightStickX, driver.rightStickY))
            put("triggers", "%.2f, %.2f".format(driver.leftTrigger, driver.rightTrigger))
        }
    }

    private companion object {
        const val SMOKE_LOOP_DELAY_MS = 20L
    }
}

private class FrameworkSmokeSubsystem(
    private val clock: Clock,
    private val event: (String) -> Unit,
) : SubsystemBase("Smoke") {

    var state = "IDLE"
        private set
    var lastEnd = "-"
        private set
    var periodicTicks = 0L
        private set
    var writeTicks = 0L
        private set
    var defaultTicks = 0L
        private set
    var workTicks = 0L
        private set
    var markerCount = 0L
        private set
    var sequenceCount = 0L
        private set
    var faultSafeCount = 0L
        private set

    override fun periodic() {
        periodicTicks++
    }

    override fun writeHardware() {
        writeTicks++
    }

    fun idleCommand(): CommandBuilder = Command.build()
        .requiring(this)
        .setStart { state = "DEFAULT" }
        .setExecute { defaultTicks++ }
        .setDone { false }
        .setEnd { state = "IDLE" }

    fun timedWorkCommand(): Command {
        var startedNs = 0L
        return Command.build()
            .requiring(this)
            .setPriority(CommandPriorities.DRIVER_ACTION)
            .setStart {
                startedNs = clock.nanos()
                state = "TIMED WORK"
                event("smoke timed work start")
            }
            .setExecute { workTicks++ }
            .setDone { clock.nanos() - startedNs >= 2_000_000_000L }
            .setEnd {
                lastEnd = "timed work: $it"
                state = "IDLE"
                event("smoke timed work end: $it")
            }
    }

    fun overrideCommand(): Command = instant {
        state = "OVERRIDE"
        event("smoke override")
    }
        .requiring(this)
        .setPriority(CommandPriorities.DRIVER_OVERRIDE)

    fun heldCommand(): Command = Command.build()
        .requiring(this)
        .setPriority(CommandPriorities.DRIVER_ACTION)
        .setStart {
            state = "HELD"
            event("smoke held command start")
        }
        .setDone { false }
        .setEnd {
            lastEnd = "held command: $it"
            state = "IDLE"
            event("smoke held command end: $it")
        }

    fun faultCommand(): Command = Command.build()
        .requiring(this)
        .setPriority(CommandPriorities.DRIVER_OVERRIDE)
        .setStart {
            state = "FAULT TEST"
            event("intentional smoke command fault requested")
        }
        .setExecute { error("intentional framework smoke-test fault") }
        .setDone { false }
        .setEnd {
            lastEnd = "fault command: $it"
            state = "IDLE"
            event("smoke fault command end: $it")
        }

    fun recordMarker() {
        markerCount++
        event("smoke marker $markerCount")
    }

    fun completeSequence() {
        sequenceCount++
        event("smoke sequence complete")
    }

    override fun onCommandFault() {
        faultSafeCount++
        state = "FAULT SAFED"
    }

    override fun health(): String = "ok; state=$state"

    override fun logState(log: StateLog) {
        log.put("state", state)
        log.put("lastEnd", lastEnd)
        log.put("periodicTicks", periodicTicks)
        log.put("writeTicks", writeTicks)
        log.put("defaultTicks", defaultTicks)
        log.put("workTicks", workTicks)
        log.put("markerCount", markerCount)
        log.put("sequenceCount", sequenceCount)
        log.put("faultSafeCount", faultSafeCount)
    }
}
