package org.firstinspires.ftc.teamcode.core.sim

import java.io.File
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.logging.WpiLog
import org.firstinspires.ftc.teamcode.core.pathing.autoRoutine
import org.firstinspires.ftc.teamcode.core.pathing.path
import org.firstinspires.ftc.teamcode.core.pathing.toPedro
import org.firstinspires.ftc.teamcode.core.runtime.PersistedPose
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Log-content regression: run a full sim auton with the real flight
 * recorder attached and assert the .wpilog tells the story — pose samples,
 * drive modes, the step-event timeline, command names, loop timings. If a
 * refactor silently stops logging something the RUNBOOK relies on, this
 * fails before a competition does.
 */
class SimAutonLogTest {

    private val startRed = Pose2d(8.0, 56.0, 0.0)
    private val outRed = Pose2d(32.0, 56.0, 0.0)
    private lateinit var logDir: File

    @Before
    fun setUp() {
        PersistedPose.storageFile = null
        PersistedPose.clear()
        logDir = File.createTempFile("sim-auton-logs", "").also {
            it.delete()
            it.mkdirs()
        }
    }

    @After
    fun tearDown() {
        PersistedPose.clear()
        logDir.deleteRecursively()
    }

    @Test
    fun simAutonProducesACompleteFlightLog() {
        val sim = SimHarness()
        sim.robot.enableFlightRecorder(
            "SimAuto",
            driver = { null },
            operator = { null },
            batteryVoltage = { 12.6 },
            directory = logDir,
        )
        sim.start()

        val outPath = sim.drive.path(startPose = startRed, alliance = Alliance.RED) {
            lineTo(outRed)
            constantHeading(0.0)
        }
        sim.follower.setStartingPose(startRed.toPedro())
        val runner = autoRoutine(sim.robot, sim.drive, sim.robot::recordEvent) {
            follow(outPath)
            holdPose(outRed)
            wait(200)
        }
        runner.schedule()
        assertTrue(sim.runUntil(10.0) { runner.isDone })
        sim.robot.stop()

        val log = WpiLog.read(logDir.listFiles { f -> f.extension == "wpilog" }!!.single())

        // Pose channel: present (one sample per tick), and it travelled
        // from start to out — the routine takes ~45 ticks of sim time.
        val poses = log.doubleArrays("pose")
        assertTrue("pose channel should have a sample per tick, got ${poses.size}", poses.size > 20)
        assertEquals(startRed.x, poses.first().second[0], 1.0)
        assertEquals(outRed.x, poses.last().second[0], 1.0)

        // Drive mode narrates the routine.
        val modes = log.strings("driveMode").map { it.second }.toSet()
        assertTrue("expected FOLLOWING in $modes", "FOLLOWING" in modes)
        assertTrue("expected HOLDING in $modes", "HOLDING" in modes)

        // Step events form the timeline.
        val events = log.strings("events").map { it.second }
        assertTrue(events.any { it == "auto step 1/3: follow" })
        assertTrue(events.any { it == "auto routine complete" })
        assertTrue(events.any { it == "stop" })

        // Running-command names are human-readable.
        val commands = log.strings("commands/running").map { it.second }
        assertTrue("expected a named auto routine in $commands", commands.any { "auto routine" in it })

        // Loop phases and follow error were recorded.
        assertTrue(log.longs("loop/totalNanos").isNotEmpty())
        assertTrue(log.has("follow/translationalErrorIn"))

        // Battery channel captured the supplied voltage.
        assertEquals(12.6, log.doubles("battery").first().second, 1e-9)
    }
}
