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
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The whole mechanism stack — profile, PIDF, soft limits, homing — running
 * headless against [SimMotorIO]. This is what the MotorIO seam buys.
 */
class ProfiledMotorSubsystemTest {

    private val clock = FakeClock()

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
    ).also { it.init(HardwareMap(null, null)) }

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
        )
        robot.scheduler.schedule(home)

        var ticks = 0
        while (robot.scheduler.isScheduled(home) && ticks < 600) {
            clock.advanceMs(20.0)
            robot.loop()
            ticks++
        }

        assertFalse("homing should complete", robot.scheduler.isScheduled(home))
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
    fun interruptedHomingHoldsInPlace() {
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

        val home = lift.homeCommand(power = -0.5, stallVelocityUnitsPerSec = 2.0)
        robot.scheduler.schedule(home)
        repeat(20) {
            clock.advanceMs(20.0)
            robot.loop()
        }
        robot.scheduler.cancel(home)
        val positionAtCancel = lift.positionUnits

        repeat(100) {
            clock.advanceMs(20.0)
            robot.loop()
        }
        assertTrue(
            "expected to hold near $positionAtCancel, got ${lift.positionUnits}",
            abs(lift.positionUnits - positionAtCancel) < 3.0,
        )
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
    fun setCurrentPositionAppliesASoftwareOffset() {
        val io = simIo(startTicks = 100.0)
        val lift = lift(io)
        lift.periodic()
        assertEquals(100.0, lift.positionUnits, 1e-9)

        lift.setCurrentPosition(10.0)
        assertEquals(10.0, lift.positionUnits, 1e-9)
        lift.periodic()
        assertEquals(10.0, lift.positionUnits, 1e-9)
    }
}
