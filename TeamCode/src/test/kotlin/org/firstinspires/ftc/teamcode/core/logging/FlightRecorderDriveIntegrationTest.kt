package org.firstinspires.ftc.teamcode.core.logging

import com.pedropathing.api.Paths
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.qualcomm.robotcore.hardware.Gamepad
import java.io.File
import java.io.IOException
import java.io.UncheckedIOException
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.PersistedPose
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.drive.PedroDriveFixture
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.core.util.GamepadEx
import org.junit.After
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * The flight recorder attached to the real Robot loop, Ivy, and the Pedro 3
 * drive: a teleop → path → driver takeover → command fault session produces a
 * WPILOG that decodes with the channels, units, transforms and timestamps
 * AdvantageScope layouts and `tools/analyze_wpilog.py` rely on.
 */
class FlightRecorderDriveIntegrationTest {

    private lateinit var logDir: File

    @Before
    fun setUp() {
        logDir = File.createTempFile("drive-integration-logs", "").also {
            it.delete()
            it.mkdirs()
        }
        PersistedPose.storageFile = null
        PersistedPose.clear()
    }

    @After
    fun tearDown() {
        logDir.deleteRecursively()
        PersistedPose.clear()
    }

    private class Session(logDir: File) {
        val hardware = PedroDriveFixture()
        val robot = Robot(hardware.hardwareMap, hardware.clock)
        val drive: MecanumDriveSubsystem = robot.register(hardware.drive)
        val localizer = robot.register(
            LocalizerSubsystem(hardware.follower, hardware.clock, onEvent = robot::recordEvent, isFollowing = drive::isFollowing),
        )
        val gamepad = Gamepad()
        val driver = GamepadEx(gamepad)
        val teleop = drive.teleopCommand { TeleopInput(driver.leftStickY, driver.leftStickX, driver.rightStickX) }

        init {
            robot.enableFlightRecorder(
                "IntegrationTest",
                driver = { driver },
                operator = { null },
                batteryVoltage = { 12.6 },
                directory = logDir,
            )
            drive.defaultCommand = teleop
            robot.start()
        }

        fun tick(control: () -> Unit = {}) {
            hardware.clock.advanceMs(10.0)
            robot.loop(input = { driver.update() }, control = control)
        }
    }

    private fun readSingleLog(): WpiLog = WpiLog.read(logDir.listFiles { f -> f.extension == "wpilog" }!!.single())

    @Test
    fun teleopPathTakeoverAndFaultProduceADecodableFieldLog() {
        val s = Session(logDir)
        s.hardware.localizer.measuredPose = Pose(8.0, 56.0, 0.0)
        s.hardware.localizer.measuredVelocity = Velocity(10.0, -2.0, 0.5)
        s.gamepad.left_stick_y = -1f
        s.gamepad.a = true
        s.gamepad.dpad_left = true
        repeat(3) { s.tick() }

        val follow = s.drive.followCommand(Paths.line(Pose(8.0, 56.0), Pose(56.0, 56.0)).constant(0.0))
        s.tick { Scheduler.schedule(follow) }
        s.hardware.localizer.measuredPose = Pose(20.0, 57.0, 0.0)
        repeat(3) { s.tick() }

        val takeover = s.drive.teleopCommand(priority = CommandPriorities.DRIVER_OVERRIDE) { TeleopInput(0.0, 0.0, 0.0) }
        s.tick { Scheduler.schedule(takeover) }
        s.tick { Scheduler.cancel(takeover) }

        val bad: Command = Command.build().requiring(s.drive).setPriority(CommandPriorities.DRIVER_ACTION).setExecute { error("boom") }
        s.tick { Scheduler.schedule(bad) }
        s.robot.recordEvent("marker")
        s.hardware.localizer.measuredPose = Pose(72.0, 72.0, Math.PI / 2)
        repeat(2) { s.tick() }
        s.robot.stop()

        val log = readSingleLog()

        // AdvantageScope 2D field: struct Pose2d in metres about the field centre.
        assertEquals("struct:Pose2d", log.type("Field/Robot"))
        assertNotNull(log.raws("/.schema/struct:Pose2d").singleOrNull())
        val field = log.structDoubles("Field/Robot")
        assertArrayEquals(doubleArrayOf(-1.6256, -0.4064, 0.0), field.first().second, 1e-9)
        assertArrayEquals(doubleArrayOf(0.0, 0.0, Math.PI / 2), field.last().second, 1e-9)

        // Raw Pedro inches, sampled on the same ticks as the struct.
        val pose = log.doubleArrays("pose")
        assertEquals(field.map { it.first }, pose.map { it.first })
        assertArrayEquals(doubleArrayOf(8.0, 56.0, 0.0), pose.first().second, 1e-9)
        assertArrayEquals(doubleArrayOf(10.0, -2.0, 0.5), log.doubleArrays("velocity").first().second, 1e-9)

        // Timestamps: microseconds since open, non-decreasing, at most one sample per 10 ms.
        val ts = pose.map { it.first }
        assertEquals(ts.sorted(), ts)
        assertTrue(ts.zipWithNext().all { (a, b) -> b - a >= 10_000 })

        val modes = log.strings("driveMode").map { it.second }.fold(mutableListOf<String>()) { acc, m ->
            if (acc.lastOrNull() != m) acc += m
            acc
        }
        assertEquals(listOf("TELEOP", "FOLLOWING", "TELEOP", "IDLE", "TELEOP"), modes)

        val modeAt = log.strings("driveMode").toMap()
        val followErrors = log.doubles("follow/translationalErrorIn")
        assertTrue(followErrors.isNotEmpty())
        assertTrue(followErrors.all { (t, _) -> modeAt[t] == "FOLLOWING" })
        assertEquals(1.0, followErrors.last().second, 1e-6)
        assertTrue(log.has("follow/headingErrorRad"))

        // Gamepad encoding: axes [lx, ly(+up), rx, ry(+up), lt, rt]; buttons bitmask A=bit0, dpadLeft=bit8.
        assertArrayEquals(doubleArrayOf(0.0, 1.0, 0.0, 0.0, 0.0, 0.0), log.doubleArrays("gamepad1/axes").first().second, 1e-9)
        assertEquals((1L shl 0) or (1L shl 8), log.longs("gamepad1/buttons").first().second)
        assertTrue(log.doubleArrays("gamepad2/axes").isEmpty())

        assertEquals(12.6, log.doubles("battery").first().second, 0.0)
        assertTrue(log.longs("loop/totalNanos").isNotEmpty())
        assertTrue(log.longs("loop/windowMaxTotalNanos").isNotEmpty())
        assertEquals(true, log.booleans("Drive/fieldCentric").first().second)
        assertEquals(false, log.booleans("Localizer/faulted").first().second)
        assertTrue(log.doubles("Drive/motors/leftFront/power").isNotEmpty())

        val events = log.strings("events").map { it.second }
        assertTrue("init IntegrationTest" in events)
        assertTrue("COMMAND FAULT: IllegalStateException: boom (commands cleared, subsystems halted)" in events)
        assertTrue("marker" in events)
        assertEquals("stop", events.last())
        assertFalse(log.has("commands/running"))
    }

    @Test
    fun recorderIoFailureDisablesRecordingButDrivingContinues() {
        val s = Session(logDir)
        var failing = false
        s.robot.register(object : SubsystemBase("Flaky") {
            override fun logState(log: StateLog) {
                if (failing) throw UncheckedIOException(IOException("sd card removed"))
                log.put("ok", true)
            }
        })
        s.gamepad.left_stick_y = -1f
        repeat(2) { s.tick() }
        failing = true
        repeat(3) { s.tick() }

        assertTrue(s.hardware.powers().all { it == 1.0 })
        assertEquals(5L, s.robot.loopCount)
        assertEquals(0, s.robot.commandFaultCount)
        s.robot.stop()

        // The file written before the failure is still a readable log.
        val log = readSingleLog()
        assertEquals(2, log.booleans("Flaky/ok").size)
    }

    @Test
    fun oldLogsArePrunedToKeepThirtyFiles() {
        repeat(31) { i ->
            File(logDir, "Old-$i.wpilog").apply {
                writeText("x")
                setLastModified(1_000_000L + i * 1_000L)
            }
        }
        val recorder = FlightRecorder.open("New", { null }, { null }, { null }, directory = logDir)
        recorder!!.close()
        val names = logDir.listFiles { f -> f.extension == "wpilog" }!!.map { it.name }
        assertEquals(30, names.size)
        assertFalse("Old-0.wpilog" in names)
        assertFalse("Old-1.wpilog" in names)
        assertTrue("Old-30.wpilog" in names)
    }

    @Test
    fun finalPoseIsPersistedAtStopAndRestoredByTheNextOpMode() {
        val auto = Session(logDir)
        auto.hardware.localizer.measuredPose = Pose(100.0, 30.0, 1.0)
        auto.tick()
        auto.robot.stop()

        val teleop = Session(logDir)
        assertTrue(teleop.localizer.restorePersistedPose())
        val restored = teleop.drive.pose
        assertEquals(100.0, restored.x(), 0.0)
        assertEquals(30.0, restored.y(), 0.0)
        assertEquals(1.0, restored.heading(), 0.0)
        teleop.robot.stop()
    }
}
