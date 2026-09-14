package org.firstinspires.ftc.teamcode.core.logging

import com.pedropathing.api.Paths
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.qualcomm.robotcore.hardware.Gamepad
import java.io.File
import java.io.IOException
import java.io.UncheckedIOException
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
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
 *
 * Set `WPILOG_SAMPLE_DIR` to keep that session's log for AdvantageScope and
 * the analyzer (see OPERATIONS.md, "Flight recorder validation").
 */
class FlightRecorderDriveIntegrationTest {

    private lateinit var logDir: File

    @Before
    fun setUp() {
        logDir = File.createTempFile("drive-integration-logs", "").also {
            it.delete()
            it.mkdirs()
        }
    }

    @After
    fun tearDown() {
        logDir.deleteRecursively()
    }

    private class Session(
        logDir: File,
        odometry: PedroDriveFixture.OdometryProbe = PedroDriveFixture.OdometryProbe(),
        registerMore: (Robot) -> Unit = {},
    ) {
        val hardware = PedroDriveFixture(localizer = odometry)
        val robot = Robot(hardware.hardwareMap, hardware.clock)
        val drive: MecanumDriveSubsystem = robot.register(hardware.drive)
        val localizer = robot.register(
            LocalizerSubsystem(hardware.follower, hardware.clock, onEvent = robot::recordEvent, isFollowing = drive::isFollowing),
        )
        val gamepad = Gamepad()
        val driver = GamepadEx(gamepad)
        val teleop = drive.teleopCommand(name = "Driver sticks") { TeleopInput(driver.leftStickY, driver.leftStickX, driver.rightStickX) }
        /** The recorder opens on this clock reading: the log's timestamp zero. */
        val openedNs = hardware.clock.nanos()

        init {
            robot.enableFlightRecorder(
                "IntegrationTest",
                driver = { driver },
                operator = { null },
                batteryVoltage = { 12.6 },
                directory = logDir,
            )
            drive.defaultCommand = teleop
            registerMore(robot)
            robot.init()
            robot.initTick()
            robot.start()
        }

        fun tick(control: () -> Unit = {}) {
            hardware.clock.advanceMs(10.0)
            robot.loop(input = { driver.update() }, control = control)
        }
    }

    private fun logFile(): File = logDir.listFiles { f -> f.extension == "wpilog" }!!.single()

    private fun readSingleLog(): WpiLog = WpiLog.read(logFile())

    private fun exportSampleIfRequested() {
        val dir = System.getenv("WPILOG_SAMPLE_DIR")?.takeIf { it.isNotBlank() } ?: return
        logFile().copyTo(File(dir, "IntegrationSample.wpilog").also { it.parentFile.mkdirs() }, overwrite = true)
    }

    @Test
    fun teleopPathTakeoverAndFaultProduceADecodableFieldLog() {
        val s = Session(logDir)
        s.hardware.localizer.measuredPose = Pose(8.0, 56.0, 0.0)
        s.hardware.localizer.measuredVelocity = Velocity(10.0, -2.0, 0.5)
        s.gamepad.left_stick_y = -1f
        s.gamepad.a = true
        s.gamepad.dpad_left = true
        repeat(3) { s.tick() }

        // A nested route: the pause after the path is never reached.
        val route = logged(
            "Route",
            sequential(
                s.drive.followCommand(Paths.line(Pose(8.0, 56.0), Pose(56.0, 56.0)).constant(0.0), name = "Follow line"),
                logged("Pause", waitUntil { false }),
            ),
        )
        s.tick { Scheduler.schedule(route) }
        s.hardware.localizer.measuredPose = Pose(20.0, 57.0, 0.0)
        repeat(3) { s.tick() }

        val takeover = s.drive.teleopCommand(priority = CommandPriorities.DRIVER_OVERRIDE, name = "Driver takeover") { TeleopInput(0.0, 0.0, 0.0) }
        s.tick { Scheduler.schedule(takeover) }
        s.hardware.localizer.measuredPose = Pose(30.0, 50.0, Math.toRadians(30.0))
        s.tick { Scheduler.cancel(takeover) }

        val bad: Command = logged(
            "Faulty step",
            Command.build().requiring(s.drive).setPriority(CommandPriorities.DRIVER_ACTION).setExecute { error("boom") },
        )
        s.tick { Scheduler.schedule(bad) }
        s.robot.recordEvent("marker")
        s.hardware.localizer.measuredPose = Pose(90.0, 40.0, Math.PI / 2)
        repeat(2) { s.tick() }
        val lastLoopUs = (s.hardware.clock.nanos() - s.openedNs) / 1000
        s.robot.stop()

        exportSampleIfRequested()
        val log = readSingleLog()

        // AdvantageScope 2D field: struct Pose2d in metres about the field
        // centre, rotated into the FTC frame by the season's quarter turns
        // (DECODE: +1, so FTC x = -(Pedro y - 72), FTC y = Pedro x - 72).
        assertEquals("struct:Pose2d", log.type("Field/Robot"))
        assertNotNull(log.raws("/.schema/struct:Pose2d").singleOrNull())
        val field = log.structDoubles("Field/Robot")
        assertArrayEquals(doubleArrayOf(0.4064, -1.6256, Math.PI / 2), field.first().second, 1e-9)
        // The last sample is the last real pose; stop adds no sample.
        assertArrayEquals(doubleArrayOf(32 * 0.0254, 18 * 0.0254, Math.PI), field.last().second, 1e-9)

        // Raw Pedro inches, sampled on the same ticks as the struct.
        val pose = log.doubleArrays("pose")
        assertEquals(field.map { it.first }, pose.map { it.first })
        assertArrayEquals(doubleArrayOf(8.0, 56.0, 0.0), pose.first().second, 1e-9)
        // Every Field/Robot sample is its pose sample moved to the field
        // centre, in metres, turned a quarter turn CCW with its heading.
        for ((i, sample) in field.withIndex()) {
            val (x, y, h) = pose[i].second
            assertArrayEquals(doubleArrayOf(-(y - 72.0) * 0.0254, (x - 72.0) * 0.0254, h + Math.PI / 2), sample.second, 1e-9)
        }
        assertEquals(
            listOf(Pose(8.0, 56.0, 0.0), Pose(20.0, 57.0, 0.0), Pose(30.0, 50.0, Math.toRadians(30.0)), Pose(90.0, 40.0, Math.PI / 2))
                .map { listOf(it.x(), it.y(), it.heading()) },
            pose.map { it.second.toList() }.distinct(),
        )
        // Shutdown: nothing sampled after the last loop, and no zero pose was invented.
        assertTrue(field.last().first <= lastLoopUs)
        assertFalse(pose.any { it.second.all { v -> v == 0.0 } })
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
        assertTrue(listOf("leftFront", "leftRear", "rightFront", "rightRear").any { log.doubles("Drive/motors/$it/power").isNotEmpty() })

        // Same-tick events get distinct timestamps, so AdvantageScope shows every one.
        val eventTimes = log.strings("events").map { it.first }
        assertTrue(eventTimes.zipWithNext().all { (a, b) -> b > a })
        val events = log.strings("events").map { it.second }
        assertTrue("init IntegrationTest" in events)
        assertTrue("COMMAND FAULT: IllegalStateException: boom (commands cleared, subsystems halted)" in events)
        assertTrue("marker" in events)
        assertEquals("stop", events.last())
        assertFalse(log.has("commands/running"))

        // Command history: what the robot was trying to do, on the same time base.
        val history = log.strings("commands/events")
        assertEquals(
            listOf(
                "START #1 Driver sticks",
                "INTERRUPT #1 Driver sticks",
                "START #2 Route",
                "START #3 Follow line",
                "INTERRUPT #3 Follow line",
                "INTERRUPT #2 Route",
                "START #4 Driver takeover",
                "INTERRUPT #4 Driver takeover",
                "START #5 Driver sticks",
                "INTERRUPT #5 Driver sticks",
                "START #6 Faulty step",
                "FAIL #6 Faulty step in execute: IllegalStateException: boom",
                "START #7 Driver sticks",
                "ABORT #7 Driver sticks: op-mode stop",
            ),
            history.map { it.second },
        )
        val historyTimes = history.map { it.first }
        assertTrue(historyTimes.zipWithNext().all { (a, b) -> b > a })
        fun at(text: String) = history.single { it.second == text }.first
        // Same clock reading as the samples: records of one tick differ only by the 1 µs
        // ordering bumps. The follow starts on the tick whose sample first shows FOLLOWING.
        val firstFollowing = log.strings("driveMode").first { it.second == "FOLLOWING" }.first
        assertTrue(abs(at("START #3 Follow line") - firstFollowing) <= 10L)
        val fault = log.strings("events").single { it.second.startsWith("COMMAND FAULT") }.first
        assertTrue(abs(fault - at("FAIL #6 Faulty step in execute: IllegalStateException: boom")) <= 10L)
        assertTrue(at("ABORT #7 Driver sticks: op-mode stop") >= lastLoopUs)

        val active = log.strings("commands/active")
        fun activeAt(t: Long) = active.last { it.first <= t }.second
        assertEquals("#2 Route\n#3 Follow line", activeAt(at("START #3 Follow line")))
        assertEquals("#4 Driver takeover", activeAt(at("START #4 Driver takeover")))
        assertEquals("", activeAt(at("FAIL #6 Faulty step in execute: IllegalStateException: boom")))
        assertEquals("", active.last().second)
        assertEquals(0L, log.longs("commands/lost").last().second)
    }

    @Test
    fun recorderIoFailureDisablesRecordingButDrivingContinues() {
        var failing = false
        val s = Session(logDir) { robot ->
            robot.register(object : SubsystemBase("Flaky") {
                override fun logState(log: StateLog) {
                    if (failing) throw UncheckedIOException(IOException("sd card removed"))
                    log.put("ok", true)
                }
            })
        }
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
    fun finalRealPoseIsKeptAtStopAndTheNextOpModeLogsFromZero() {
        val first = Session(logDir)
        first.hardware.localizer.measuredPose = Pose(100.0, 30.0, 1.0)
        first.tick()
        first.robot.stop()
        val firstLog = readSingleLog()
        assertArrayEquals(doubleArrayOf(100.0, 30.0, 1.0), firstLog.doubleArrays("pose").last().second, 1e-9)
        logDir.listFiles()!!.forEach { it.delete() }

        // Same Pinpoint, still holding the first run's pose.
        val second = Session(logDir, first.hardware.localizer)
        assertArrayEquals(DoubleArray(3), second.drive.pose.let { doubleArrayOf(it.x(), it.y(), it.heading()) }, 0.0)
        assertTrue(second.localizer.ready)
        second.tick()
        second.robot.stop()
        val secondLog = readSingleLog()
        assertTrue(secondLog.doubleArrays("pose").all { it.second.contentEquals(doubleArrayOf(0.0, 0.0, 0.0)) })
    }
}
