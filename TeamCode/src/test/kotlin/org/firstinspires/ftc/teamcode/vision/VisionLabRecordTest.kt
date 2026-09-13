package org.firstinspires.ftc.teamcode.vision

import java.io.File
import java.util.concurrent.Executor
import org.firstinspires.ftc.teamcode.core.runtime.ConfigStore
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class VisionLabRecordTest {

    private lateinit var tempDir: File
    private var originalFile: File? = null

    @Before
    fun setUp() {
        tempDir = createTempDirectory()
        originalFile = ConfigStore.file
        ConfigStore.reset()
        ConfigStore.file = File(tempDir, "tuning.properties")
        BallVisionConfig.resetDefaults()
        VisionDiagnosticsConfig.resetDefaults()
    }

    @After
    fun tearDown() {
        ConfigStore.reset()
        ConfigStore.file = originalFile
        BallVisionConfig.resetDefaults()
        VisionDiagnosticsConfig.resetDefaults()
        tempDir.deleteRecursively()
    }

    @Test
    fun recordUsesTuningFileKeysAndFlagsOnlyTunedValuesWithTheirAdoptionEdit() {
        ConfigStore.register("ballVision", BallVisionConfig, BallVisionConfig::resetDefaults)
        ConfigStore.register("visionDiagnostics", VisionDiagnosticsConfig, VisionDiagnosticsConfig::resetDefaults)
        BallVisionConfig.channel1Min = 140
        BallVisionConfig.minCircularity = 0.72
        VisionDiagnosticsConfig.runBothCameras = true

        val text = VisionLabRecord.build(
            opModeName = "Ball Tracking Test",
            wallClockMs = 0L,
            configSchema = "schema-x",
            sections = listOf(
                VisionLabRecord.sectionFromStore("ballVision", BallVisionConfig.compiledDefaults()),
                VisionLabRecord.sectionFromStore("visionDiagnostics", VisionDiagnosticsConfig.compiledDefaults()),
            ),
            measurements = listOf("camera.processedFps" to "87.5"),
        )
        val lines = text.lines()

        assertTrue(lines.any { it.startsWith("ballVision.channel1Min=140") && "TUNED; compiled default 128" in it })
        assertTrue(lines.any { it.startsWith("ballVision.minCircularity=") && "compiled default 0.5" in it })
        assertTrue(lines.any { it.startsWith("ballVision.channel1Max=170") && "TUNED" !in it })
        assertTrue("ballVision: DEFAULT_CHANNEL1_MIN = 140" in lines)
        assertTrue("ballVision: DEFAULT_MIN_CIRCULARITY = 0.72" in lines)
        assertTrue("visionDiagnostics: DEFAULT_RUN_BOTH_CAMERAS = true" in lines)
        assertTrue("camera.processedFps=87.5" in lines)
        assertTrue(text.contains("Config schema: schema-x"))

        // Keys in the record are exactly what ConfigStore persists, so a tuned file can be diffed against it.
        val persisted = ConfigStore.snapshot()
        assertEquals(persisted["ballVision.channel1Min"], "140")
    }

    @Test
    fun untunedRecordSaysNothingToAdopt() {
        ConfigStore.register("ballVision", BallVisionConfig, BallVisionConfig::resetDefaults)
        val text = VisionLabRecord.build(
            "Limelight AprilTag Test", 0L, "s",
            listOf(VisionLabRecord.sectionFromStore("ballVision", BallVisionConfig.compiledDefaults())),
            emptyList(),
        )
        assertTrue(text.contains("# nothing differs from the compiled defaults"))
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
