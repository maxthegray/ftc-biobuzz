package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.sim.SimMotorIO
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Before
import org.junit.Test

class IntakeSubsystemTest {
    private val savedConfig = ConfigSnapshot(IntakeConfig)
    private val clock = FakeClock()
    private val motors = listOf(SimMotorIO(clock), SimMotorIO(clock))
    private val intake = IntakeSubsystem { motors }

    @Before
    fun setUp() {
        Scheduler.reset()
        intake.init(HardwareMap(null, null))
    }

    @After
    fun tearDown() {
        Scheduler.reset()
        savedConfig.restore()
    }

    private fun tick() {
        Scheduler.execute()
        intake.writeHardware()
    }

    @Test
    fun collectAndEjectDriveEveryMotorAndStopWhenInterrupted() {
        val collect = intake.collect()
        Scheduler.schedule(collect)
        tick()
        motors.forEach { assertEquals(IntakeConfig.collectPower, it.lastPower, 0.0) }

        Scheduler.schedule(intake.eject())
        tick()
        motors.forEach { assertEquals(IntakeConfig.ejectPower, it.lastPower, 0.0) }

        Scheduler.reset()
        Scheduler.schedule(collect)
        Scheduler.cancel(collect)
        intake.writeHardware()
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }
    }

    @Test
    fun faultsAndStopZeroTheMotors() {
        Scheduler.schedule(intake.collect())
        tick()
        intake.onCommandFault()
        intake.writeHardware()
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }

        Scheduler.reset()
        Scheduler.schedule(intake.collect())
        tick()
        intake.stop()
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }
    }
}
