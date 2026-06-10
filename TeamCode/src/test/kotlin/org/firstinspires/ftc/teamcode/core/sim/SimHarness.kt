package org.firstinspires.ftc.teamcode.core.sim

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.core.util.FakeClock

/**
 * Virtual-time harness that runs the real framework stack — [Robot]'s
 * phased loop, the Ivy scheduler, [MecanumDriveSubsystem],
 * [LocalizerSubsystem] — against a [SimFollower], so entire auton routines
 * execute headless in a JUnit test in milliseconds.
 *
 * The caller owns `Scheduler.reset()` in test setup/teardown, exactly like
 * the other framework tests.
 */
class SimHarness(tickMs: Double = 20.0) {
    val clock = FakeClock(start = 0L)
    val follower = SimFollower(clock)
    val robot = Robot(HardwareMap(null, null), clock)
    val drive: MecanumDriveSubsystem = robot.register(MecanumDriveSubsystem(follower))
    val localizer: LocalizerSubsystem = robot.register(LocalizerSubsystem(follower, clock))

    private val tickMs = tickMs

    fun start() {
        robot.init()
        robot.start()
    }

    /** Advance one sim tick: time first, then a full robot loop. */
    fun tick(control: () -> Unit = {}) {
        clock.advanceMs(tickMs)
        robot.loop(control = control)
    }

    /**
     * Tick until [condition] holds or [maxSimSeconds] of *simulated* time
     * elapses. Returns whether the condition was met.
     */
    fun runUntil(maxSimSeconds: Double, condition: () -> Boolean): Boolean {
        var elapsedMs = 0.0
        while (elapsedMs < maxSimSeconds * 1000.0) {
            if (condition()) return true
            tick()
            elapsedMs += tickMs
        }
        return condition()
    }
}
