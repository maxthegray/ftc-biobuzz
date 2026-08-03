package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerConfig
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The TeleOpBase watchdog policy, wired the same way against fakes: a
 * localizer fault while the teleop default command is the *active* command
 * must not leave the sticks dead. `breakPath()` clears Pedro's manual-drive
 * mode, and an already-scheduled default never re-runs its start — so the
 * policy cancels it and lets the next tick reschedule it.
 */
class TeleopWatchdogRecoveryTest {

    @Test
    fun watchdogFaultDuringStickDrivingReenablesManualDrive() {
        LocalizerConfig.watchdogEnabled = true
        val clock = FakeClock()
        val robot = Robot(HardwareMap(null, null), clock)
        val follower = fakeFollower()
        val drive = robot.register(MecanumDriveSubsystem(follower, clock))
        val teleopDefault = drive.teleopCommand { MecanumDriveSubsystem.TeleopInput(0.0, 0.0, 0.0) }
        drive.defaultCommand = teleopDefault
        robot.register(
            LocalizerSubsystem(
                follower,
                clock = clock,
                isFollowing = drive::isFollowing,
                onFault = {
                    drive.breakPath()
                    robot.scheduler.cancel(teleopDefault)
                },
            ),
        )

        robot.start()
        clock.advanceMs(20.0)
        robot.loop()
        assertEquals(1, follower.startTeleopDriveCalls)
        assertTrue(robot.scheduler.isScheduled(teleopDefault))

        // The localizer dies: a non-finite pose trips the watchdog while the
        // teleop default is the active command.
        follower.setPose(Pose(Double.NaN, 0.0, 0.0))
        clock.advanceMs(20.0)
        robot.loop()

        assertEquals(2, follower.startTeleopDriveCalls)
        assertTrue(robot.scheduler.isScheduled(teleopDefault))
    }
}
