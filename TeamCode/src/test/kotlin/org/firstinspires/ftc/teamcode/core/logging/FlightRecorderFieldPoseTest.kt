package org.firstinspires.ftc.teamcode.core.logging

import com.qualcomm.robotcore.hardware.HardwareMap
import java.io.File
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.Vector2d
import org.firstinspires.ftc.teamcode.core.runtime.DriveTelemetrySource
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Test

/**
 * End-to-end: a recorded op-mode produces a log AdvantageScope's 2D Field tab
 * can draw a robot from. Covers the seam between the recorder and
 * [WpiStruct] — that the struct channel is declared with the right type, that
 * its schemas ride along in the same log, and that it tracks the drive's pose
 * tick for tick alongside the untouched Pedro-inch `pose` channel.
 */
class FlightRecorderFieldPoseTest {

    private class FakeDrive(override var pose: Pose2d) : SubsystemBase("Drive"), DriveTelemetrySource {
        override val velocity: Vector2d = Vector2d(0.0, 0.0)
        override val driveModeName: String = "TELEOP"
        override val isPathing: Boolean = false
        override val angularVelocityRadPerSec: Double = 0.0
        override val followTranslationalErrorInches: Double = Double.NaN
        override val followHeadingErrorRad: Double = Double.NaN
        override fun currentPathPoses(samplesPerPath: Int): List<List<Pose2d>> = emptyList()
    }

    @Test
    fun fieldRobotChannelIsAStructTrackingTheDrivePose() = withLogDir { logDir ->
        val clock = FakeClock()
        val robot = Robot(HardwareMap(null, null), clock)
        val drive = FakeDrive(Pose2d(8.0, 56.0, 0.0))
        robot.register(drive)
        robot.enableFlightRecorder(
            "FieldPoseTest",
            driver = { null },
            operator = { null },
            batteryVoltage = { 12.5 },
            directory = logDir,
        )

        robot.start()
        robot.loop()
        clock.advanceMs(20.0)
        drive.pose = Pose2d(72.0, 72.0, Math.PI / 2)
        robot.loop()
        robot.stop()

        val log = read(logDir)

        // The type string is what makes AdvantageScope treat the bytes as a
        // pose at all; "raw" would decode to nothing.
        assertEquals("struct:Pose2d", log.type("Field/Robot"))
        assertNotNull(log.raws("/.schema/struct:Pose2d").singleOrNull())

        val poses = log.structDoubles("Field/Robot")
        assertEquals(2, poses.size)

        // (8, 56) inches from the corner -> (-64, -16) inches from the centre.
        assertEquals(-1.6256, poses[0].second[0], 1e-9)
        assertEquals(-0.4064, poses[0].second[1], 1e-9)
        assertEquals(0.0, poses[0].second[2], 0.0)

        // Field centre encodes as the origin, heading untouched.
        assertEquals(0.0, poses[1].second[0], 1e-9)
        assertEquals(0.0, poses[1].second[1], 1e-9)
        assertEquals(Math.PI / 2, poses[1].second[2], 0.0)

        // Both channels are sampled from the same tick, so the struct and the
        // raw inches must agree pose for pose and timestamp for timestamp.
        val inches = log.doubleArrays("pose")
        assertEquals(inches.map { it.first }, poses.map { it.first })
        assertEquals(8.0, inches[0].second[0], 0.0)
        assertEquals(56.0, inches[0].second[1], 0.0)
    }

    @Test
    fun everyStructPayloadIsExactlyOnePose() = withLogDir { logDir ->
        val clock = FakeClock()
        val robot = Robot(HardwareMap(null, null), clock)
        robot.register(FakeDrive(Pose2d(24.0, 24.0, 0.0)))
        robot.enableFlightRecorder(
            "FieldPoseSizeTest",
            driver = { null },
            operator = { null },
            batteryVoltage = { null },
            directory = logDir,
        )

        robot.start()
        repeat(5) {
            robot.loop()
            clock.advanceMs(20.0)
        }
        robot.stop()

        val payloads = read(logDir).raws("Field/Robot")
        assertEquals(5, payloads.size)
        // A short or long payload decodes as garbage rather than failing, so
        // the reused buffer must never leak a partial write.
        payloads.forEach { assertEquals(WpiStruct.POSE2D_SIZE, it.second.size) }
    }

    private fun read(logDir: File): WpiLog =
        WpiLog.read(logDir.listFiles { f -> f.extension == "wpilog" }!!.single())

    private fun withLogDir(block: (File) -> Unit) {
        val logDir = File.createTempFile("field-pose-logs", "").also {
            it.delete()
            it.mkdirs()
        }
        try {
            block(logDir)
        } finally {
            logDir.deleteRecursively()
        }
    }
}
