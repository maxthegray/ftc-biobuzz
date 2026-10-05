package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Scheduler
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Before
import org.junit.Test

class IntakeSubsystemTest {
    private val savedConfig = ConfigSnapshot(IntakeConfig)
    private val motors = RobotConfig.Intake.MOTORS.map { MotorProbe() }
    private val intake = IntakeSubsystem()

    @Before
    fun setUp() {
        Scheduler.reset()
        val hardwareMap = FakeHardwareMap()
        RobotConfig.Intake.MOTORS.forEachIndexed { i, name -> hardwareMap.put(name, motors[i].device) }
        intake.init(hardwareMap)
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
        motors.forEach { assertEquals(IntakeConfig.collectPower, it.power, 0.0) }

        Scheduler.schedule(intake.eject())
        tick()
        motors.forEach { assertEquals(IntakeConfig.ejectPower, it.power, 0.0) }

        Scheduler.reset()
        Scheduler.schedule(collect)
        Scheduler.cancel(collect)
        intake.writeHardware()
        motors.forEach { assertEquals(0.0, it.power, 0.0) }
    }

    @Test
    fun faultsAndStopZeroTheMotors() {
        Scheduler.schedule(intake.collect())
        tick()
        intake.onCommandFault()
        intake.writeHardware()
        motors.forEach { assertEquals(0.0, it.power, 0.0) }

        Scheduler.reset()
        Scheduler.schedule(intake.collect())
        tick()
        intake.stop()
        motors.forEach { assertEquals(0.0, it.power, 0.0) }
    }
}
