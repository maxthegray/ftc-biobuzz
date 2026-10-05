package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.behaviors.EndCondition
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.firstinspires.ftc.teamcode.core.sim.ServoProbe
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class TransferSubsystemTest {
    private val savedConfig = ConfigSnapshot(TransferConfig)
    private val clock = FakeClock()
    private val motors = RobotConfig.Transfer.MOTORS.map { MotorProbe() }
    private val blocker = ServoProbe()
    private val transfer = TransferSubsystem(clock)

    private fun hardwareWith(blocker: ServoProbe) = FakeHardwareMap().apply {
        RobotConfig.Transfer.MOTORS.forEachIndexed { i, name -> put(name, motors[i].device) }
        put(RobotConfig.Transfer.BLOCKER_SERVO, blocker.device)
    }

    @Before
    fun setUp() {
        Scheduler.reset()
        transfer.init(hardwareWith(blocker))
    }

    @After
    fun tearDown() {
        Scheduler.reset()
        savedConfig.restore()
    }

    private fun tick(ms: Double = 20.0) {
        clock.advanceMs(ms)
        Scheduler.execute()
        transfer.writeHardware()
    }

    private fun assertSafe() {
        motors.forEach { assertEquals(0.0, it.power, 0.0) }
        assertEquals(TransferConfig.blockerClosedPosition, blocker.position, 0.0)
    }

    @Test
    fun theDefaultHoldsBallsBehindTheClosedBlocker() {
        Scheduler.schedule(transfer.defaultCommand!!)
        tick()
        assertSafe()
    }

    @Test
    fun stagingPushesBallsAgainstTheClosedBlocker() {
        Scheduler.schedule(transfer.stage())
        tick()
        motors.forEach { assertEquals(TransferConfig.stagePower, it.power, 0.0) }
        assertEquals(TransferConfig.blockerClosedPosition, blocker.position, 0.0)
    }

    @Test
    fun feedingWaitsForTheBlockerToSwingClear() {
        val feed = transfer.feed()
        Scheduler.schedule(feed)
        tick(0.0)
        assertEquals(TransferConfig.blockerOpenPosition, blocker.position, 0.0)
        motors.forEach { assertEquals(0.0, it.power, 0.0) }

        tick(TransferConfig.blockerTravelMs - 1.0)
        assertFalse(transfer.blockerSettled)
        motors.forEach { assertEquals(0.0, it.power, 0.0) }

        tick(1.0)
        assertTrue(transfer.blockerSettled)
        motors.forEach { assertEquals(TransferConfig.feedPower, it.power, 0.0) }

        Scheduler.cancel(feed)
        transfer.writeHardware()
        assertSafe()
    }

    @Test
    fun aDelayedFirstWriteDoesNotConsumeTheBlockerTravelTime() {
        transfer.writeHardware()
        Scheduler.schedule(transfer.feed())
        clock.advanceMs(TransferConfig.blockerTravelMs + 50.0)
        Scheduler.execute()
        assertFalse(transfer.blockerSettled)
        assertEquals(TransferConfig.blockerClosedPosition, blocker.position, 0.0)

        transfer.writeHardware()
        assertEquals(TransferConfig.blockerOpenPosition, blocker.position, 0.0)
        motors.forEach { assertEquals(0.0, it.power, 0.0) }

        tick(TransferConfig.blockerTravelMs - 1.0)
        assertFalse(transfer.blockerSettled)
        motors.forEach { assertEquals(0.0, it.power, 0.0) }

        tick(1.0)
        assertTrue(transfer.blockerSettled)
        motors.forEach { assertEquals(TransferConfig.feedPower, it.power, 0.0) }
    }

    @Test
    fun theTravelTimerStartsAfterTheServoWriteReturns() {
        val slowBlocker = ServoProbe { from, to -> if (to != from) clock.advanceMs(TransferConfig.blockerTravelMs) }
        val slowTransfer = TransferSubsystem(clock)
        slowTransfer.init(hardwareWith(slowBlocker))
        Scheduler.schedule(slowTransfer.feed())
        Scheduler.execute()
        slowTransfer.writeHardware()
        assertFalse(slowTransfer.blockerSettled)

        clock.advanceMs(TransferConfig.blockerTravelMs - 1.0)
        Scheduler.execute()
        slowTransfer.writeHardware()
        motors.forEach { assertEquals(0.0, it.power, 0.0) }

        clock.advanceMs(1.0)
        Scheduler.execute()
        slowTransfer.writeHardware()
        motors.forEach { assertEquals(TransferConfig.feedPower, it.power, 0.0) }
    }

    @Test
    fun aSecondFeedWaitsForTheBlockerAgain() {
        val feed = transfer.feed()
        Scheduler.schedule(feed)
        tick(0.0)
        tick(TransferConfig.blockerTravelMs)
        motors.forEach { assertEquals(TransferConfig.feedPower, it.power, 0.0) }
        Scheduler.cancel(feed)
        Scheduler.schedule(transfer.feed())
        tick(0.0)
        motors.forEach { assertEquals(0.0, it.power, 0.0) }

        tick(TransferConfig.blockerTravelMs - 1.0)
        motors.forEach { assertEquals(0.0, it.power, 0.0) }
        tick(1.0)
        motors.forEach { assertEquals(TransferConfig.feedPower, it.power, 0.0) }
    }

    @Test
    fun anEndWithoutAStartLeavesTheRunningCommandAlone() {
        val stage = transfer.stage()
        Scheduler.schedule(stage)
        tick()
        transfer.feed().end(EndCondition.INTERRUPTED)
        tick()
        motors.forEach { assertEquals(TransferConfig.stagePower, it.power, 0.0) }
    }

    @Test
    fun faultsAndStopMakeTheTransferSafe() {
        Scheduler.schedule(transfer.feed())
        tick(0.0)
        tick(TransferConfig.blockerTravelMs)
        transfer.onCommandFault()
        transfer.writeHardware()
        assertSafe()

        Scheduler.reset()
        Scheduler.schedule(transfer.reverse())
        tick()
        transfer.stop()
        motors.forEach { assertEquals(0.0, it.power, 0.0) }
        assertFalse(transfer.blockerOpen)
    }
}
