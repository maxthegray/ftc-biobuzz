package org.firstinspires.ftc.teamcode.subsystems.transfer

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.behaviors.EndCondition
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.io.ServoIO
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.sim.FakeServoIO
import org.firstinspires.ftc.teamcode.core.sim.SimMotorIO
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class TransferSubsystemTest {
    private val clock = FakeClock()
    private val motors = listOf(SimMotorIO(clock), SimMotorIO(clock))
    private val blocker = FakeServoIO()
    private val transfer = TransferSubsystem(clock) { TransferSubsystem.Hardware(motors, blocker) }

    @Before
    fun setUp() {
        Scheduler.reset()
        TransferConfig.resetDefaults()
        transfer.init(HardwareMap(null, null))
    }

    @After
    fun tearDown() {
        Scheduler.reset()
        TransferConfig.resetDefaults()
    }

    private fun tick(ms: Double = 20.0) {
        clock.advanceMs(ms)
        Scheduler.execute()
        transfer.writeHardware()
    }

    private fun assertSafe() {
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }
        assertEquals(TransferConfig.blockerClosedPosition, blocker.lastPosition, 0.0)
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
        motors.forEach { assertEquals(TransferConfig.stagePower, it.lastPower, 0.0) }
        assertEquals(TransferConfig.blockerClosedPosition, blocker.lastPosition, 0.0)
    }

    @Test
    fun feedingWaitsForTheBlockerToSwingClear() {
        val feed = transfer.feed()
        Scheduler.schedule(feed)
        tick(0.0)
        assertEquals(TransferConfig.blockerOpenPosition, blocker.lastPosition, 0.0)
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }

        tick(TransferConfig.blockerTravelMs - 1.0)
        assertFalse(transfer.blockerSettled)
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }

        tick(1.0)
        assertTrue(transfer.blockerSettled)
        motors.forEach { assertEquals(TransferConfig.feedPower, it.lastPower, 0.0) }

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
        assertEquals(TransferConfig.blockerClosedPosition, blocker.lastPosition, 0.0)

        transfer.writeHardware()
        assertEquals(TransferConfig.blockerOpenPosition, blocker.lastPosition, 0.0)
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }

        tick(TransferConfig.blockerTravelMs - 1.0)
        assertFalse(transfer.blockerSettled)
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }

        tick(1.0)
        assertTrue(transfer.blockerSettled)
        motors.forEach { assertEquals(TransferConfig.feedPower, it.lastPower, 0.0) }
    }

    @Test
    fun theTravelTimerStartsAfterTheServoWriteReturns() {
        val slowBlocker = object : ServoIO {
            override val lastPosition: Double get() = blocker.lastPosition

            override fun setPosition(position: Double) {
                if (position != lastPosition) clock.advanceMs(TransferConfig.blockerTravelMs)
                blocker.setPosition(position)
            }
        }
        val slowTransfer = TransferSubsystem(clock) { TransferSubsystem.Hardware(motors, slowBlocker) }
        slowTransfer.init(HardwareMap(null, null))
        Scheduler.schedule(slowTransfer.feed())
        Scheduler.execute()
        slowTransfer.writeHardware()
        assertFalse(slowTransfer.blockerSettled)

        clock.advanceMs(TransferConfig.blockerTravelMs - 1.0)
        Scheduler.execute()
        slowTransfer.writeHardware()
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }

        clock.advanceMs(1.0)
        Scheduler.execute()
        slowTransfer.writeHardware()
        motors.forEach { assertEquals(TransferConfig.feedPower, it.lastPower, 0.0) }
    }

    @Test
    fun aSecondFeedWaitsForTheBlockerAgain() {
        val feed = transfer.feed()
        Scheduler.schedule(feed)
        tick(0.0)
        tick(TransferConfig.blockerTravelMs)
        motors.forEach { assertEquals(TransferConfig.feedPower, it.lastPower, 0.0) }
        Scheduler.cancel(feed)
        Scheduler.schedule(transfer.feed())
        tick(0.0)
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }

        tick(TransferConfig.blockerTravelMs - 1.0)
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }
        tick(1.0)
        motors.forEach { assertEquals(TransferConfig.feedPower, it.lastPower, 0.0) }
    }

    @Test
    fun anEndWithoutAStartLeavesTheRunningCommandAlone() {
        val stage = transfer.stage()
        Scheduler.schedule(stage)
        tick()
        transfer.feed().end(EndCondition.INTERRUPTED)
        tick()
        motors.forEach { assertEquals(TransferConfig.stagePower, it.lastPower, 0.0) }
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
        motors.forEach { assertEquals(0.0, it.lastPower, 0.0) }
        assertFalse(transfer.blockerOpen)
    }

    @Test
    fun outOfRangeSettingsAreClamped() {
        TransferConfig.feedPower = 3.0
        TransferConfig.blockerOpenPosition = Double.NaN
        TransferConfig.blockerTravelMs = -5.0
        Scheduler.schedule(transfer.feed())
        tick(0.0)
        tick(150.0)
        motors.forEach { assertEquals(1.0, it.lastPower, 0.0) }
        assertEquals(0.5, blocker.lastPosition, 0.0)
    }
}
