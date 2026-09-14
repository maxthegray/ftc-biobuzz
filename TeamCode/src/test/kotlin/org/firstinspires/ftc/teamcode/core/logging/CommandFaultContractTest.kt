package org.firstinspires.ftc.teamcode.core.logging

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.behaviors.EndCondition
import java.io.File
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.drive.PedroDriveFixture
import org.junit.After
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * The fault contract: an [Exception] from a command is contained (COMMAND
 * FAULT, everything halted, loop continues); an [Error] such as `TODO()`'s
 * [NotImplementedError] escapes the loop, and the op-mode's crash path stops
 * every actuator and closes the log.
 */
class CommandFaultContractTest {

    /** A mechanism with its own motor, written only in writeHardware. */
    private class Arm : SubsystemBase("Arm") {
        var target = 0.0
        var applied = 0.0
        val ends = mutableListOf<EndCondition>()

        fun raise(execute: () -> Unit = {}): Command = Command.build()
            .requiring(this)
            .setPriority(CommandPriorities.DRIVER_ACTION)
            .setExecute { target = 0.6; execute() }
            .setEnd { ends += it; target = 0.0 }

        override fun writeHardware() { applied = target }
        override fun onCommandFault() { target = 0.0; applied = 0.0 }
        override fun stop() { target = 0.0; applied = 0.0 }
    }

    private lateinit var logDir: File
    private val hardware = PedroDriveFixture()
    private val robot = Robot(hardware.hardwareMap, hardware.clock)
    private val drive = robot.register(hardware.drive)
    private val arm = robot.register(Arm())

    @Before
    fun setUp() {
        logDir = File.createTempFile("fault-contract", "").also { it.delete(); it.mkdirs() }
        robot.enableFlightRecorder("FaultContract", { null }, { null }, { 12.5 }, directory = logDir)
        drive.defaultCommand = drive.teleopCommand { TeleopInput(1.0, 0.0, 0.0) }
        robot.init()
        robot.initTick()
        robot.start()
        tick()
        Scheduler.schedule(arm.raise())
        tick()
        assertTrue(hardware.powers().all { it > 0.0 })
        assertEquals(0.6, arm.applied, 0.0)
    }

    @After
    fun tearDown() {
        robot.stop()
        logDir.deleteRecursively()
    }

    private fun tick() {
        hardware.clock.advanceMs(20.0)
        robot.loop()
    }

    private fun events(): List<String> =
        WpiLog.read(logDir.listFiles { f -> f.extension == "wpilog" }!!.single()).strings("events").map { it.second }

    @Test
    fun anExceptionFromACommandIsContainedAndZeroesEveryActuatorForATick() {
        val bad = Command.build().requiring(Any()).setExecute { throw IllegalArgumentException("bad setpoint") }
        Scheduler.schedule(bad)
        tick()

        assertEquals(1, robot.commandFaultCount)
        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
        assertEquals(0.0, arm.applied, 0.0)
        // Aborted, not ended: no end handler ran.
        assertTrue(arm.ends.isEmpty())
        assertFalse(Scheduler.isScheduled(bad))

        tick()
        tick()
        assertTrue("drive default resumed", hardware.powers().all { it > 0.0 })
        assertEquals(0.0, arm.applied, 0.0)

        robot.stop()
        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
        val events = events()
        assertTrue("COMMAND FAULT: IllegalArgumentException: bad setpoint (commands cleared, subsystems halted)" in events)
        assertEquals("stop", events.last())
    }

    @Test
    fun todoInACommandEscapesTheLoopAndTheCrashPathStopsEverything() {
        val unfinished = arm.raise { TODO() }
        Scheduler.schedule(unfinished)

        val crash = assertThrows(NotImplementedError::class.java) { tick() }
        assertEquals(0, robot.commandFaultCount)
        // The loop did not get to write or halt anything: the op-mode's crash path must.
        assertTrue(hardware.powers().all { it > 0.0 })

        val reported = mutableListOf<String>()
        robot.stopAfterCrash(crash) { reported += it }

        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
        assertEquals(0.0, arm.applied, 0.0)
        assertFalse(Scheduler.isScheduled(unfinished))
        assertEquals(listOf("LOOP CRASHED: NotImplementedError: An operation is not implemented."), reported)
        val events = events()
        val crashEvent = events.single { it.startsWith("LOOP CRASHED") }
        assertTrue(crashEvent.contains("kotlin.NotImplementedError"))
        assertTrue(crashEvent.contains("CommandFaultContractTest"))
        assertEquals("stop", events.last())
        assertThrows(IllegalStateException::class.java) { tick() }
    }

    @Test
    fun todoInABindingEscapesTooButAnExceptionThereIsContained() {
        hardware.clock.advanceMs(20.0)
        robot.loop(input = { error("binding bug") })
        assertEquals(1, robot.commandFaultCount)

        hardware.clock.advanceMs(20.0)
        val crash = assertThrows(NotImplementedError::class.java) { robot.loop(input = { TODO("binding") }) }
        robot.stopAfterCrash(crash)
        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
        assertEquals(0.0, arm.applied, 0.0)
        assertTrue(events().any { it.startsWith("LOOP CRASHED: NotImplementedError: An operation is not implemented: binding") })
    }
}
