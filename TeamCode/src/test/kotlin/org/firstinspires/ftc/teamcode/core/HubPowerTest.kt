package org.firstinspires.ftc.teamcode.core

import com.qualcomm.robotcore.hardware.HardwareMap
import java.io.File
import org.firstinspires.ftc.teamcode.core.logging.WpiLog
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class HubPowerTest {

    private class FakeHub(override val label: String, var amps: Double, var volts: Double) : HubPower.Hub {
        val reads = mutableListOf<String>()
        override fun amps(): Double = amps.also { reads += "amps" }
        override fun volts(): Double = volts.also { reads += "volts" }
    }

    @Test
    fun oneHubIsReadEveryIntervalRoundRobinCurrentThenVoltage() {
        val clock = FakeClock()
        val control = FakeHub("control", 4.0, 12.5)
        val expansion = FakeHub("expansion2", 7.0, 12.4)
        val power = HubPower(listOf(control, expansion), clock)

        power.sample()
        clock.advanceMs(20.0)
        power.sample()
        assertEquals(listOf("amps", "volts"), control.reads)
        assertTrue(expansion.reads.isEmpty())
        assertEquals(12.5, power.batteryVolts, 0.0)

        clock.advanceMs(30.0)
        power.sample()
        assertEquals(listOf("amps", "volts"), expansion.reads)
        assertEquals(7.0, power.readings[1].amps, 0.0)

        control.volts = 11.9
        clock.advanceMs(50.0)
        power.sample()
        assertEquals(11.9, power.batteryVolts, 0.0)
    }

    @Test
    fun aFailingHubLeavesItsReadingEmptyAndTheNextHubIsStillRead() {
        val clock = FakeClock()
        val broken = object : HubPower.Hub {
            override val label = "control"
            override fun amps(): Double = throw IllegalStateException("lost")
            override fun volts(): Double = 12.0
        }
        val expansion = FakeHub("expansion2", 3.0, 12.2)
        val power = HubPower(listOf(broken, expansion), clock)

        power.sample()
        clock.advanceMs(50.0)
        power.sample()
        assertTrue(power.batteryVolts.isNaN())
        assertEquals(false, power.readings[0].unlogged)
        assertEquals(3.0, power.readings[1].amps, 0.0)
    }

    @Test
    fun aZeroVoltReadingIsDropped() {
        val clock = FakeClock()
        val control = FakeHub("control", 4.0, 12.5)
        val power = HubPower(listOf(control), clock)

        power.sample()
        power.readings[0].unlogged = false
        control.amps = 0.0
        control.volts = 0.0
        clock.advanceMs(50.0)
        power.sample()
        assertEquals(12.5, power.batteryVolts, 0.0)
        assertEquals(4.0, power.readings[0].amps, 0.0)
        assertEquals(false, power.readings[0].unlogged)
    }

    @Test
    fun theFlightRecorderLogsEachReadingOnceEvenFromATickItDidNotRecord() {
        val logDir = File.createTempFile("hub-power-logs", "").also {
            it.delete()
            it.mkdirs()
        }
        try {
            val clock = FakeClock()
            val robot = Robot(HardwareMap(null, null), clock)
            val control = FakeHub("control", 5.0, 12.6)
            robot.hubPower = HubPower(listOf(control), clock)
            robot.enableFlightRecorder("HubTest", { null }, { null }, { null }, directory = logDir)

            robot.initTick()
            clock.advanceMs(10.0)
            robot.start()
            robot.loop()
            control.amps = 9.0
            control.volts = 12.1
            for (i in 0 until 10) {
                clock.advanceMs(6.0)
                robot.loop()
            }
            robot.stop()

            val log = WpiLog.read(logDir.listFiles { f -> f.extension == "wpilog" }!!.single())
            assertEquals(listOf(5.0, 9.0), log.doubles("hubs/control/currentAmps").map { it.second })
            assertEquals(listOf(12.6, 12.1), log.doubles("hubs/control/volts").map { it.second })
            assertEquals(
                log.doubles("hubs/control/currentAmps").map { it.first },
                log.doubles("hubs/control/volts").map { it.first },
            )
        } finally {
            logDir.deleteRecursively()
        }
    }
}
