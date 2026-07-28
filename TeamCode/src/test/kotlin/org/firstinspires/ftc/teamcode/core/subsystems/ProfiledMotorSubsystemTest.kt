package org.firstinspires.ftc.teamcode.core.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.core.control.PIDFGains
import org.firstinspires.ftc.teamcode.core.control.ProfileConstraints
import org.firstinspires.ftc.teamcode.core.control.ProfiledController
import org.firstinspires.ftc.teamcode.core.hw.SimMotorIO
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * The whole mechanism stack — profile, PIDF, soft limits, homing — running
 * headless against [SimMotorIO]. This is what the MotorIO seam buys.
 */
class ProfiledMotorSubsystemTest {

    private val clock = FakeClock()

    @Before
    fun clearHomingHandoff() {
        ProfiledMotorSubsystem.clearPersistedHomingForTest()
    }

    private fun controller() = ProfiledController(
        ProfileConstraints(maxVelocity = 50.0, maxAcceleration = 150.0),
        PIDFGains(kP = 0.4, kV = 0.01),
    )

    private fun simIo(
        startTicks: Double = 0.0,
        minTicks: Double = Double.NEGATIVE_INFINITY,
        maxTicks: Double = Double.POSITIVE_INFINITY,
    ) = SimMotorIO(
        clock,
        freeSpeedTicksPerSec = 100.0,
        timeConstantSec = 0.05,
        minPositionTicks = minTicks,
        maxPositionTicks = maxTicks,
    ).also { it.setPositionTicks(startTicks) }

    private fun lift(
        io: SimMotorIO,
        softMin: Double? = null,
        softMax: Double? = null,
    ): ProfiledMotorSubsystem = ProfiledMotorSubsystem(
        name = "Lift",
        motorName = "unused-in-sim",
        controller = controller(),
        ticksPerUnit = 1.0,
        softMinUnits = softMin,
        softMaxUnits = softMax,
        io = io,
        clock = clock,
    ).also {
        it.init(HardwareMap(null, null))
        it.periodic()
        it.setCurrentPosition(it.positionUnits)
    }

    private fun tick(subsystem: ProfiledMotorSubsystem, ms: Double = 20.0) {
        clock.advanceMs(ms)
        subsystem.periodic()
        subsystem.writeHardware()
    }

    @Test
    fun closedLoopReachesTheGoal() {
        val io = simIo()
        val lift = lift(io)

        lift.setGoal(24.0)
        repeat(200) { tick(lift) } // 4 simulated seconds

        assertTrue("expected ~24, got ${lift.positionUnits}", abs(lift.positionUnits - 24.0) < 1.0)
        assertTrue(lift.atGoal(toleranceUnits = 1.0))
    }

    @Test
    fun onCommandFaultFreezesAClosedLoopMoveInPlace() {
        val io = simIo()
        val lift = lift(io)

        lift.setGoal(24.0)
        repeat(25) { tick(lift) } // 0.5 s — mid-travel
        val positionAtFault = lift.positionUnits
        assertTrue("expected mid-travel, got $positionAtFault", positionAtFault < 20.0)

        lift.onCommandFault()
        repeat(200) { tick(lift) }

        // Frozen where the fault happened — not at the abandoned goal, and
        // not de-energized (health still shows closed-loop hold).
        assertTrue(
            "expected to hold near $positionAtFault, got ${lift.positionUnits}",
            abs(lift.positionUnits - positionAtFault) < 2.0,
        )
        assertTrue(lift.health().contains("CLOSED_LOOP"))
    }

    @Test
    fun goalsClampToSoftLimits() {
        val io = simIo()
        val lift = lift(io, softMin = 0.0, softMax = 10.0)

        lift.setGoal(50.0)
        repeat(250) { tick(lift) }

        assertTrue("expected ≤ ~10, got ${lift.positionUnits}", lift.positionUnits < 11.0)
        // The clamped goal is reachable, so the mechanism settles at the limit.
        assertTrue(abs(lift.positionUnits - 10.0) < 1.0)
    }

    @Test
    fun openLoopPowerPastAViolatedSoftLimitIsZeroed() {
        val io = simIo(startTicks = 12.0)
        val lift = lift(io, softMax = 10.0)
        lift.periodic() // read the out-of-bounds position

        lift.openLoop(0.5)
        tick(lift)
        assertEquals("power into the limit must be zeroed", 0.0, io.lastPower, 1e-12)

        lift.openLoop(-0.5)
        tick(lift)
        assertTrue("power back into bounds must pass", io.lastPower < 0.0)
    }

    @Test
    fun homingFindsTheHardStopAndRezeroes() {
        // Hard stop at -30 ticks; mechanism thinks it starts at 50.
        val io = simIo(startTicks = 50.0, minTicks = -30.0)
        val robot = Robot(HardwareMap(null, null), clock)
        val lift = ProfiledMotorSubsystem(
            name = "Lift",
            motorName = "unused-in-sim",
            controller = controller(),
            ticksPerUnit = 1.0,
            io = io,
            clock = clock,
        )
        robot.register(lift)
        robot.init()
        robot.start()

        val home = lift.homeCommand(
            power = -0.5,
            stallVelocityUnitsPerSec = 2.0,
            stallTimeMs = 100.0,
            graceMs = 100.0,
            resetToUnits = 0.0,
            timeoutMs = 3_000.0,
        )
        robot.scheduler.schedule(home)

        var ticks = 0
        while (robot.scheduler.isScheduled(home) && ticks < 600) {
            clock.advanceMs(20.0)
            robot.loop()
            ticks++
        }

        assertFalse("homing should complete", robot.scheduler.isScheduled(home))
        assertEquals(ProfiledMotorSubsystem.HomingState.HOMED, lift.homingState)
        // The hard stop (raw -30) is now defined as 0.
        assertTrue("expected ~0 at the stop, got ${lift.positionUnits}", abs(lift.positionUnits) < 1.0)

        // And the mechanism holds closed-loop at the new zero.
        repeat(50) {
            clock.advanceMs(20.0)
            robot.loop()
        }
        assertTrue(abs(lift.positionUnits) < 1.5)

        // Goals are now in the homed frame.
        lift.setGoal(20.0)
        repeat(200) {
            clock.advanceMs(20.0)
            robot.loop()
        }
        assertTrue("expected ~20 homed units, got ${lift.positionUnits}", abs(lift.positionUnits - 20.0) < 1.5)
    }

    @Test
    fun interruptedHomingFromUnknownZeroDisablesWithoutDeclaringHome() {
        val io = simIo(startTicks = 50.0, minTicks = -30.0)
        val robot = Robot(HardwareMap(null, null), clock)
        val lift = ProfiledMotorSubsystem(
            name = "Lift",
            motorName = "unused-in-sim",
            controller = controller(),
            ticksPerUnit = 1.0,
            io = io,
            clock = clock,
        )
        robot.register(lift)
        robot.init()
        robot.start()

        val home = lift.homeCommand(
            power = -0.5,
            stallVelocityUnitsPerSec = 2.0,
            timeoutMs = 3_000.0,
        )
        robot.scheduler.schedule(home)
        repeat(20) {
            clock.advanceMs(20.0)
            robot.loop()
        }
        robot.scheduler.cancel(home)
        clock.advanceMs(20.0)
        robot.loop()

        assertEquals(ProfiledMotorSubsystem.HomingState.UNHOMED, lift.homingState)
        assertEquals(0.0, io.lastPower, 0.0)
    }

    @Test
    fun commandFaultDuringOpenLoopFreezesInPlace() {
        val io = simIo(startTicks = 30.0)
        val lift = lift(io)
        lift.periodic()
        lift.openLoop(0.3)
        tick(lift)

        lift.onCommandFault()

        assertTrue(lift.atGoal(toleranceUnits = 2.0))
        val held = lift.positionUnits
        repeat(100) { tick(lift) }
        assertTrue(
            "expected to hold near $held, got ${lift.positionUnits}",
            abs(lift.positionUnits - held) < 2.0,
        )
    }

    @Test
    fun goToCommandTimeoutEndsAStalledMoveAndKeepsHolding() {
        // Hard stop at 10 ticks; the goal is unreachable, so the move stalls.
        val io = simIo(maxTicks = 10.0)
        val robot = Robot(HardwareMap(null, null), clock)
        val lift = ProfiledMotorSubsystem(
            name = "Lift",
            motorName = "unused-in-sim",
            controller = controller(),
            ticksPerUnit = 1.0,
            io = io,
            clock = clock,
        )
        robot.register(lift)
        robot.init()
        lift.periodic()
        lift.setCurrentPosition(lift.positionUnits)
        robot.start()

        val blocked = lift.goToCommand(20.0, toleranceUnits = 0.5, timeoutMs = 500.0)
        robot.scheduler.schedule(blocked)

        repeat(20) { // 400 ms — still inside the timeout
            clock.advanceMs(20.0)
            robot.loop()
        }
        assertTrue(robot.scheduler.isScheduled(blocked))

        repeat(10) { // past 500 ms — the timeout releases the step
            clock.advanceMs(20.0)
            robot.loop()
        }
        assertFalse(robot.scheduler.isScheduled(blocked))
        assertEquals(ProfiledMotorSubsystem.GoalOutcome.TIMED_OUT, lift.lastGoalOutcome)

        // The subsystem keeps holding the (unreached) goal closed-loop:
        // pinned against the stop, not dropping.
        repeat(50) {
            clock.advanceMs(20.0)
            robot.loop()
        }
        assertTrue(
            "expected to stay pressed near the stop, got ${lift.positionUnits}",
            lift.positionUnits > 8.0,
        )
    }

    @Test
    fun goToCommandWithoutTimeoutStaysScheduledWhileStalled() {
        val io = simIo(maxTicks = 10.0)
        val robot = Robot(HardwareMap(null, null), clock)
        val lift = ProfiledMotorSubsystem(
            name = "Lift",
            motorName = "unused-in-sim",
            controller = controller(),
            ticksPerUnit = 1.0,
            io = io,
            clock = clock,
        )
        robot.register(lift)
        robot.init()
        lift.periodic()
        lift.setCurrentPosition(lift.positionUnits)
        robot.start()

        val blocked = lift.goToCommand(20.0, toleranceUnits = 0.5)
        robot.scheduler.schedule(blocked)
        repeat(100) { // 2 simulated seconds
            clock.advanceMs(20.0)
            robot.loop()
        }
        assertTrue(robot.scheduler.isScheduled(blocked))
    }

    @Test
    fun setCurrentPositionAppliesASoftwareOffset() {
        val io = simIo(startTicks = 100.0)
        val lift = lift(io)
        lift.periodic()
        assertEquals(100.0, lift.positionUnits, 1e-9)

        lift.setCurrentPosition(10.0)
        assertEquals(10.0, lift.positionUnits, 1e-9)
        lift.periodic()
        assertEquals(10.0, lift.positionUnits, 1e-9)
        assertEquals(ProfiledMotorSubsystem.HomingState.HOMED, lift.homingState)
    }

    @Test
    fun unhomedMechanismAllowsOpenLoopButRejectsClosedLoopGoals() {
        val io = simIo(startTicks = 12.0)
        val lift = ProfiledMotorSubsystem(
            name = "Lift",
            motorName = "unhomed",
            controller = controller(),
            ticksPerUnit = 1.0,
            softMaxUnits = 10.0,
            io = io,
            clock = clock,
        )
        lift.init(HardwareMap(null, null))
        lift.periodic()

        assertThrows(IllegalStateException::class.java) { lift.setGoal(5.0) }

        // The arbitrary pre-home coordinate must not activate the soft limit.
        lift.openLoop(0.25)
        tick(lift)
        assertTrue(io.lastPower > 0.0)
        assertEquals(ProfiledMotorSubsystem.HomingState.UNHOMED, lift.homingState)
    }

    @Test
    fun homingTimeoutFaultsAndDisablesInsteadOfDeclaringZero() {
        val io = simIo(startTicks = 0.0)
        val robot = Robot(HardwareMap(null, null), clock)
        robot.containCommandFaults = true
        val lift = ProfiledMotorSubsystem(
            name = "Lift",
            motorName = "timeout",
            controller = controller(),
            ticksPerUnit = 1.0,
            io = io,
            clock = clock,
        )
        robot.register(lift)
        robot.init()
        robot.start()
        val home = lift.homeCommand(
            power = 0.5,
            stallVelocityUnitsPerSec = 0.1,
            stallTimeMs = 50.0,
            graceMs = 50.0,
            timeoutMs = 300.0,
        )
        assertTrue(robot.scheduler.schedule(home))

        repeat(20) {
            clock.advanceMs(20.0)
            robot.loop()
        }

        assertFalse(robot.scheduler.isScheduled(home))
        assertEquals(1, robot.commandFaultCount)
        assertTrue(robot.lastCommandFault is ProfiledMotorSubsystem.HomingTimeoutException)
        assertEquals(ProfiledMotorSubsystem.HomingState.FAULTED, lift.homingState)
        assertEquals(0.0, io.lastPower, 0.0)
    }

    @Test
    fun successfulHomePersistsItsCoordinateFrameIntoTheNextOpMode() {
        val io = simIo(startTicks = 20.0, minTicks = -10.0)
        val autoRobot = Robot(HardwareMap(null, null), clock)
        val autoLift = ProfiledMotorSubsystem(
            name = "Lift",
            motorName = "handoff",
            controller = controller(),
            ticksPerUnit = 1.0,
            io = io,
            clock = clock,
        )
        autoRobot.register(autoLift)
        autoRobot.init()
        autoRobot.start()
        val home = autoLift.homeCommand(
            power = -0.5,
            stallVelocityUnitsPerSec = 2.0,
            stallTimeMs = 100.0,
            graceMs = 100.0,
            timeoutMs = 2_000.0,
        )
        autoRobot.scheduler.schedule(home)
        repeat(150) {
            clock.advanceMs(20.0)
            autoRobot.loop()
            if (!autoRobot.scheduler.isScheduled(home)) return@repeat
        }
        assertEquals(ProfiledMotorSubsystem.HomingState.HOMED, autoLift.homingState)
        autoRobot.stop()

        val teleopRobot = Robot(HardwareMap(null, null), clock)
        val teleopLift = ProfiledMotorSubsystem(
            name = "Lift",
            motorName = "handoff",
            controller = controller(),
            ticksPerUnit = 1.0,
            io = io,
            clock = clock,
        )
        teleopRobot.register(teleopLift)
        teleopRobot.init()
        teleopLift.periodic()

        assertEquals(ProfiledMotorSubsystem.HomingState.HOMED, teleopLift.homingState)
        assertTrue("expected restored zero, got ${teleopLift.positionUnits}", abs(teleopLift.positionUnits) < 1.0)
    }

    @Test
    fun nonFiniteFinalPowerFaultsAndWritesZero() {
        val io = simIo()
        val lift = lift(io)

        lift.openLoop(Double.NaN)
        tick(lift)

        assertEquals(0.0, io.lastPower, 0.0)
        assertEquals(ProfiledMotorSubsystem.HomingState.FAULTED, lift.homingState)
    }

    @Test
    fun goalTimeoutCanOptIntoDisablingOutput() {
        val io = simIo(maxTicks = 10.0)
        val robot = Robot(HardwareMap(null, null), clock)
        val lift = lift(io)
        robot.register(lift)
        robot.start()
        val command = lift.goToCommand(
            20.0,
            toleranceUnits = 0.5,
            timeoutMs = 300.0,
            timeoutPolicy = ProfiledMotorSubsystem.GoalTimeoutPolicy.DISABLE,
        )
        robot.scheduler.schedule(command)

        repeat(20) {
            clock.advanceMs(20.0)
            robot.loop()
        }

        assertEquals(ProfiledMotorSubsystem.GoalOutcome.TIMED_OUT, lift.lastGoalOutcome)
        assertEquals(0.0, io.lastPower, 0.0)
    }
}
