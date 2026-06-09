package org.firstinspires.ftc.teamcode.core.runtime

import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Base class every subsystem extends.
 *
 * A subsystem owns a slice of the robot (drive, intake, shooter, lift…) and
 * presents a safe, high-level API. The main loop is single-threaded: the
 * OpMode calls [periodic] on every subsystem once per tick, after the Lynx
 * bulk read but before the command scheduler tick.
 *
 * Subsystems double as Ivy "requirements" — pass `this` to
 * `CommandBuilder.requiring(...)` and the scheduler will automatically
 * resolve conflicts between commands that touch the same hardware.
 */
abstract class SubsystemBase(val name: String) {

    /**
     * Called exactly once when the OpMode initialises. Resolve hardware,
     * zero encoders, apply motor directions. Failures should throw — the
     * [Robot] catches them and reports via telemetry.
     */
    open fun init(hardwareMap: HardwareMap) {}

    /**
     * Called every main-loop tick, after the bulk read completes and before
     * the scheduler executes commands. Use this for pure reads / state
     * updates — not for commanding actuators. Command hardware from commands
     * so the scheduler can arbitrate conflicts.
     */
    open fun periodic() {}

    /**
     * Called every main-loop tick after [periodic] and the scheduler tick.
     * Useful for writing the final motor power / servo position decided by
     * whichever command is currently running.
     */
    open fun writeHardware() {}

    /**
     * Called once at the end of the OpMode. Zero motors, stop threads,
     * release any resources. Never throw from here.
     *
     * This is the safety net for shutdown: the scheduler reset that runs
     * first does NOT call `end()` on in-flight commands, so don't rely on
     * command cleanup having happened — leave the hardware safe
     * unconditionally.
     */
    open fun stop() {}

    override fun toString(): String = "Subsystem($name)"
}
