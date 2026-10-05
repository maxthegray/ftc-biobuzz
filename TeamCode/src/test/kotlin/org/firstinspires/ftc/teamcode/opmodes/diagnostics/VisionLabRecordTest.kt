package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import org.firstinspires.ftc.teamcode.vision.ball.BallCameraConfig
import java.io.File
import java.util.concurrent.Executor
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class VisionLabRecordTest {

    private lateinit var tempDir: File
    private val savedBall = ConfigSnapshot(BallCameraConfig)
    private val savedDiagnostics = ConfigSnapshot(VisionDiagnosticsConfig)

    @Before
    fun setUp() {
        tempDir = createTempDirectory()
    }

    @After
    fun tearDown() {
        savedBall.restore()
        savedDiagnostics.restore()
        tempDir.deleteRecursively()
    }

    private fun sections() = listOf(
        VisionLabRecord.section(BallCameraConfig, BallCameraConfig.compiledDefaults),
        VisionLabRecord.section(VisionDiagnosticsConfig, VisionDiagnosticsConfig.compiledDefaults),
    )

    @Test
    fun recordFlagsOnlyTunedValuesWithTheirAdoptionEdit() {
        BallCameraConfig.channel1Min = 140
        BallCameraConfig.minCircularity = 0.72
        VisionDiagnosticsConfig.runBothCameras = true

        val text = VisionLabRecord.build(
            opModeName = "Ball Tracking Test",
            wallClockMs = 0L,
            sections = sections(),
            measurements = listOf("camera.processedFps" to "87.5"),
        )
        val lines = text.lines()

        assertTrue(lines.any { it.startsWith("BallCameraConfig.channel1Min=140") && "TUNED; compiled value 130" in it })
        assertTrue(lines.any { it.startsWith("BallCameraConfig.minCircularity=0.72") && "compiled value 1.0" in it })
        assertTrue(lines.any { it.startsWith("BallCameraConfig.channel1Max=170") && "TUNED" !in it })
        assertTrue("BallCameraConfig.channel1Min = 140" in lines)
        assertTrue("BallCameraConfig.minCircularity = 0.72" in lines)
        assertTrue("VisionDiagnosticsConfig.runBothCameras = true" in lines)
        assertTrue("camera.processedFps=87.5" in lines)
    }

    @Test
    fun untunedRecordSaysNothingToAdopt() {
        val text = VisionLabRecord.build("Limelight AprilTag Test", 0L, sections(), emptyList())
        assertTrue(text.contains("# nothing differs from the compiled values"))
        assertFalse(text.contains("TUNED"))
    }

    @Test
    fun fileNamesAreSlugged() {
        val name = VisionLabRecord.fileName("Limelight AprilTag Test", 0L)
        assertTrue(name.matches(Regex("limelight-apriltag-test-\\d{8}-\\d{6}\\.txt")))
    }

    @Test
    fun writerWritesAtomicallyAndReportsStatus() {
        val directExecutor = Executor { it.run() }
        val records = File(tempDir, "lab-records")
        val writer = LabRecordWriter(records, directExecutor)

        writer.submit("a.txt", "hello")

        assertEquals("hello", File(records, "a.txt").readText())
        assertFalse(File(records, "a.txt.tmp").exists())
        assertEquals(1, writer.status.written)
        assertEquals(0, writer.status.pending)
    }

    @Test
    fun writerFailureIsReportedNotThrown() {
        val blocker = File(tempDir, "not-a-directory").apply { writeText("x") }
        val writer = LabRecordWriter(blocker, Executor { it.run() })

        writer.submit("a.txt", "hello")

        assertNotNull(writer.status.lastError)
        assertEquals(0, writer.status.written)
        assertEquals(0, writer.status.pending)
    }

    private fun createTempDirectory(): File =
        File.createTempFile("vision-lab", "").apply {
            delete()
            mkdirs()
        }
}
