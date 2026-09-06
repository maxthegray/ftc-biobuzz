package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.subsystems.drive.PedroDriveFixture
import org.firstinspires.ftc.teamcode.core.subsystems.drive.deviceProxy
import org.junit.Assert.*
import org.junit.Test
import java.nio.ByteBuffer
import java.nio.ByteOrder

class PinpointReadinessTest {
    private class Harness {
        val hardware = PedroDriveFixture()
        val clock = FakeClock()
        var deviceStatusBits = 0
        var failReads = false
        val pinpoint = GoBildaPinpointDriver(deviceProxy(I2cDeviceSynchSimple::class.java) { name, args ->
            if (name == "read") {
                check(!failReads) { "I2C unavailable" }
                val bytes = ByteBuffer.allocate(args[1] as Int).order(ByteOrder.LITTLE_ENDIAN)
                when (args[0] as Int) {
                    2 -> bytes.putInt(2) // Firmware v2, fixed 40-byte bulk packet.
                    18 -> { bytes.putInt(deviceStatusBits); bytes.putInt(1000) }
                    else -> error("Unexpected Pinpoint register ${args[0]}")
                }
                bytes.array()
            } else null
        }, false)
        var faults = 0
        val localizer = LocalizerSubsystem(hardware.follower, clock, onFault = { faults++ })
        val robot = Robot(hardware.hardwareMap, clock)

        init {
            hardware.hardwareMap.put(RobotConfig.Localization.PINPOINT, pinpoint)
            hardware.localizer.onRead = { pinpoint.update() }
            robot.register(hardware.drive)
            robot.register(localizer)
            robot.init()
            hardware.clearWrites()
        }

        fun initTick(ms: Double = 20.0) {
            clock.advanceMs(ms)
            robot.initTick()
        }
    }

    @Test
    fun freshSdkStatusIsRefreshedBeforeTheWatchdogJudgesIt() {
        val h = Harness()
        h.deviceStatusBits = 1 // The device is ready; the SDK cache is still NOT_READY.
        assertEquals(DeviceStatus.NOT_READY, h.pinpoint.deviceStatus)
        assertFalse(h.localizer.ready)

        h.initTick()

        assertEquals(DeviceStatus.READY, h.pinpoint.deviceStatus)
        assertTrue(h.localizer.ready)
        assertNull(h.localizer.fault)
        assertEquals(1, h.hardware.localizer.reads)
        assertTrue(h.hardware.motors.all { it.writes.isEmpty() })
    }

    @Test
    fun calibrationIsAllowedDuringInitButDoesNotDeclareReadiness() {
        val h = Harness()
        h.initTick()
        assertFalse(h.localizer.ready)
        assertNull(h.localizer.fault)
        h.deviceStatusBits = 2
        h.initTick(100.0)
        assertFalse(h.localizer.ready)
        assertNull(h.localizer.fault)
        assertTrue(h.localizer.health().contains("waiting"))

        h.deviceStatusBits = 1
        h.initTick(100.0)
        assertTrue(h.localizer.ready)
        assertEquals(0, h.faults)
        assertTrue(h.hardware.motors.all { it.writes.isEmpty() })
    }

    @Test
    fun startupTimeoutUsesElapsedTimeAndLatchesOnce() {
        val h = Harness()
        h.deviceStatusBits = 2
        h.initTick(4999.0)
        assertNull(h.localizer.fault)
        h.initTick(1.0)
        assertTrue(h.localizer.fault!!.contains("timed out"))
        h.deviceStatusBits = 1
        h.initTick()
        assertFalse(h.localizer.ready)
        assertEquals(1, h.faults)
    }

    @Test
    fun missingPodsTripImmediatelyEvenDuringStartupGrace() {
        val h = Harness()
        h.deviceStatusBits = 12
        h.initTick()
        assertTrue(h.localizer.fault!!.contains("FAULT_NO_PODS_DETECTED"))
        assertFalse(h.localizer.ready)
        assertEquals(1, h.faults)
    }

    @Test
    fun startingBeforeReadyDoesNotReceiveAnActiveRunGracePeriod() {
        val h = Harness()
        h.deviceStatusBits = 2
        h.initTick()
        assertFalse(h.localizer.ready)
        assertNull(h.localizer.fault)
        h.robot.start()
        h.localizer.periodic()
        assertTrue(h.localizer.fault!!.contains("CALIBRATING"))
    }

    @Test
    fun losingReadyDuringInitIsAFaultInsteadOfRestartingTheGracePeriod() {
        val h = Harness()
        h.deviceStatusBits = 1
        h.initTick()
        h.deviceStatusBits = 0
        h.initTick()
        assertTrue(h.localizer.fault!!.contains("NOT_READY"))
    }

    @Test
    fun initReadFailureIsVisibleAndDoesNotWriteMotorPower() {
        val h = Harness()
        h.failReads = true
        h.initTick()
        assertTrue(h.localizer.fault!!.contains("I2C unavailable"))
        assertFalse(h.localizer.ready)
        assertTrue(h.hardware.motors.all { it.writes.isEmpty() })
    }

    @Test
    fun activeLoopUsesOnlyTheDriveUpdateToRefreshTheSensor() {
        val h = Harness()
        h.deviceStatusBits = 1
        h.initTick()
        h.robot.start()
        h.clock.advanceMs(20.0)
        h.robot.loop()
        assertEquals(2, h.hardware.localizer.reads)
        assertNull(h.localizer.fault)
    }
}
