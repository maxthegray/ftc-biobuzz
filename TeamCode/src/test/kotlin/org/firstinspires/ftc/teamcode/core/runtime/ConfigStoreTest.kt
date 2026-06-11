package org.firstinspires.ftc.teamcode.core.runtime

import java.io.File
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class ConfigStoreTest {

    // Kotlin object @JvmField vars compile to static fields — the exact
    // shape Panels tunes and ConfigStore must round-trip.
    private object TestTuning {
        @JvmField var gain: Double = 0.5
        @JvmField var enabled: Boolean = false
        @JvmField var count: Int = 3
        @JvmField var label: String = "default"
    }

    private lateinit var tempFile: File
    private var originalFile: File? = null

    @Before
    fun setUp() {
        tempFile = File.createTempFile("tuning", ".properties").also { it.delete() }
        originalFile = ConfigStore.file
        ConfigStore.reset()
        ConfigStore.file = tempFile
        TestTuning.gain = 0.5
        TestTuning.enabled = false
        TestTuning.count = 3
        TestTuning.label = "default"
    }

    @After
    fun tearDown() {
        ConfigStore.reset()
        ConfigStore.file = originalFile
        tempFile.delete()
    }

    @Test
    fun roundTripsTunedValuesAcrossAProcessRestart() {
        ConfigStore.register("test", TestTuning)
        ConfigStore.loadFromDisk()

        // "Panels tunes some values mid-match."
        TestTuning.gain = 0.875
        TestTuning.enabled = true
        TestTuning.count = 7
        TestTuning.label = "tuned"
        assertTrue(ConfigStore.persistIfDirty())

        // "Power cycle": statics reset to compiled defaults, store reloads.
        TestTuning.gain = 0.5
        TestTuning.enabled = false
        TestTuning.count = 3
        TestTuning.label = "default"
        ConfigStore.reset()
        ConfigStore.file = tempFile
        ConfigStore.register("test", TestTuning)
        ConfigStore.loadFromDisk()

        assertEquals(0.875, TestTuning.gain, 0.0)
        assertTrue(TestTuning.enabled)
        assertEquals(7, TestTuning.count)
        assertEquals("tuned", TestTuning.label)
    }

    @Test
    fun cleanStateDoesNotRewriteTheFile() {
        ConfigStore.register("test", TestTuning)
        ConfigStore.loadFromDisk()
        assertFalse("nothing changed — no write expected", ConfigStore.persistIfDirty())
        assertFalse(tempFile.exists())

        TestTuning.gain = 0.6
        assertTrue(ConfigStore.persistIfDirty())
        assertTrue(tempFile.exists())
        assertFalse("already persisted — no second write", ConfigStore.persistIfDirty())
    }

    @Test
    fun unknownKeysAndGarbageLinesAreIgnored() {
        tempFile.writeText(
            """
            # comment
            not a key value pair
            test.doesNotExist=42
            otherSection.gain=9.9
            test.gain=0.75
            """.trimIndent(),
        )
        ConfigStore.register("test", TestTuning)
        ConfigStore.loadFromDisk()

        assertEquals(0.75, TestTuning.gain, 0.0)
        assertEquals(3, TestTuning.count)
    }

    @Test
    fun invalidValuesFallBackToCompiledDefaults() {
        tempFile.writeText(
            """
            test.gain=NaN
            test.count=not-a-number
            test.enabled=maybe
            """.trimIndent(),
        )
        ConfigStore.register("test", TestTuning)
        ConfigStore.loadFromDisk()

        assertEquals(0.5, TestTuning.gain, 0.0)
        assertEquals(3, TestTuning.count)
        assertFalse(TestTuning.enabled)
    }

    @Test
    fun missingFileLeavesDefaultsUntouched() {
        ConfigStore.register("test", TestTuning)
        ConfigStore.loadFromDisk()
        assertEquals(0.5, TestTuning.gain, 0.0)
    }

    @Test
    fun snapshotKeysAreSectionQualified() {
        ConfigStore.register("test", TestTuning)
        val snapshot = ConfigStore.snapshot()
        assertTrue("test.gain" in snapshot)
        assertTrue("test.enabled" in snapshot)
        assertTrue("test.count" in snapshot)
        assertTrue("test.label" in snapshot)
    }

    @Test
    fun frameworkConfigObjectsExposeTheirTunables() {
        ConfigStore.register("drive", org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig)
        ConfigStore.register(
            "localizer",
            org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerConfig,
        )
        val snapshot = ConfigStore.snapshot()
        assertTrue("drive.inputExponent" in snapshot)
        assertTrue("drive.fieldCentric" in snapshot)
        assertTrue("localizer.correctionBlend" in snapshot)
        assertTrue("localizer.followingBlendScale" in snapshot)
        // Private defaults and synthetic fields must not leak.
        assertFalse(snapshot.keys.any { "DEFAULT" in it || "INSTANCE" in it })
    }

    @Test
    fun doubleRoundTripIsExact() {
        ConfigStore.register("test", TestTuning)
        TestTuning.gain = 1.0 / 3.0
        ConfigStore.persistIfDirty()

        TestTuning.gain = 0.0
        ConfigStore.reset()
        ConfigStore.file = tempFile
        ConfigStore.register("test", TestTuning)
        ConfigStore.loadFromDisk()

        assertEquals(1.0 / 3.0, TestTuning.gain, 0.0)
    }
}
