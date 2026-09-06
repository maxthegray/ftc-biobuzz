package org.firstinspires.ftc.teamcode.core.runtime

import com.bylazar.configurables.annotations.Configurable
import java.io.File
import java.lang.reflect.Modifier
import org.firstinspires.ftc.teamcode.core.control.PIDFGains
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerConfig
import org.firstinspires.ftc.teamcode.opmodes.diagnostics.MotorTestConfig
import org.firstinspires.ftc.teamcode.vision.BallAimConfig
import org.firstinspires.ftc.teamcode.vision.BallApproachConfig
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/** Stand-in for a non-primitive live holder synced from primitive config fields. */
private class TestConstraints(
    @JvmField var maxVelocity: Double,
    @JvmField var maxAcceleration: Double,
)

class ConfigStoreTest {

    // Kotlin object @JvmField vars compile to static fields — the exact
    // shape Panels tunes and ConfigStore must round-trip.
    private object TestTuning {
        @JvmField var gain: Double = 0.5
        @JvmField var enabled: Boolean = false
        @JvmField var count: Int = 3
        @JvmField var label: String = "default"

        fun resetDefaults() {
            gain = 0.5
            enabled = false
            count = 3
            label = "default"
        }
    }

    @Configurable
    private object MechanismTuning {
        @JvmField var kP: Double = 0.1
        @JvmField var kV: Double = 0.02
        @JvmField var maxVelocity: Double = 30.0
        @JvmField var maxAcceleration: Double = 60.0

        val gains = PIDFGains()
        val constraints = TestConstraints(1.0, 1.0)

        fun resetDefaults() {
            kP = 0.1
            kV = 0.02
            maxVelocity = 30.0
            maxAcceleration = 60.0
        }

        fun sync() {
            gains.kP = kP
            gains.kV = kV
            constraints.maxVelocity = maxVelocity
            constraints.maxAcceleration = maxAcceleration
        }
    }

    private lateinit var tempFile: File
    private var originalFile: File? = null

    @Before
    fun setUp() {
        tempFile = File.createTempFile("tuning", ".properties").also { it.delete() }
        originalFile = ConfigStore.file
        ConfigStore.reset()
        ConfigStore.file = tempFile
        TestTuning.resetDefaults()
        MechanismTuning.resetDefaults()
        MechanismTuning.sync()
    }

    @After
    fun tearDown() {
        ConfigStore.reset()
        ConfigStore.file = originalFile
        tempFile.deleteRecursively()
        File(tempFile.path + ".tmp").deleteRecursively()
    }

    @Test
    fun roundTripsTunedValuesAcrossAProcessRestart() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
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
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()

        assertEquals(0.875, TestTuning.gain, 0.0)
        assertTrue(TestTuning.enabled)
        assertEquals(7, TestTuning.count)
        assertEquals("tuned", TestTuning.label)
    }

    @Test
    fun cleanStateDoesNotRewriteTheFile() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
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
            $schemaLine
            # comment
            not a key value pair
            test.doesNotExist=42
            otherSection.gain=9.9
            test.gain=0.75
            """.trimIndent(),
        )
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()

        assertEquals(0.75, TestTuning.gain, 0.0)
        assertEquals(3, TestTuning.count)
    }

    @Test
    fun invalidValuesFallBackToCompiledDefaults() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        TestTuning.gain = 0.9
        TestTuning.count = 9
        TestTuning.enabled = true
        TestTuning.label = "previous opmode"
        tempFile.writeText(
            """
            $schemaLine
            test.gain=NaN
            test.count=not-a-number
            test.enabled=maybe
            """.trimIndent(),
        )
        ConfigStore.loadFromDisk()

        assertEquals(0.5, TestTuning.gain, 0.0)
        assertEquals(3, TestTuning.count)
        assertFalse(TestTuning.enabled)
        assertEquals("default", TestTuning.label)
    }

    @Test
    fun deletingFileRestoresDefaultsOnNextWarmInit() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        TestTuning.gain = 0.9
        TestTuning.enabled = true
        assertTrue(ConfigStore.persistIfDirty())

        assertTrue(tempFile.delete())
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()

        assertEquals(0.5, TestTuning.gain, 0.0)
        assertFalse(TestTuning.enabled)
        assertFalse(ConfigStore.persistIfDirty())
        assertFalse(tempFile.exists())
    }

    @Test
    fun tuningBeforeFirstRegistrationDoesNotBecomeTheDefault() {
        TestTuning.gain = 0.9
        TestTuning.enabled = true
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)

        ConfigStore.loadFromDisk()

        assertEquals(0.5, TestTuning.gain, 0.0)
        assertFalse(TestTuning.enabled)
    }

    @Test
    fun missingKeyRestoresDefaultInsteadOfLastOpmodeValue() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        tempFile.writeText("$schemaLine\ntest.gain=0.9\ntest.count=9\n")
        ConfigStore.loadFromDisk()
        assertEquals(9, TestTuning.count)

        tempFile.writeText("$schemaLine\ntest.gain=0.75\n")
        ConfigStore.loadFromDisk()

        assertEquals(0.75, TestTuning.gain, 0.0)
        assertEquals(3, TestTuning.count)
    }

    @Test
    fun partialRegistrationPreservesOtherOpmodesTuningAcrossRestart() {
        tempFile.writeText("$schemaLine\ntest.gain=0.75\nlift.kP=0.8\ntest.futureField=42\n")
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        TestTuning.count = 8
        assertTrue(ConfigStore.persistIfDirty())
        assertTrue(tempFile.readLines().contains("test.futureField=42"))

        ConfigStore.reset()
        ConfigStore.register("lift", MechanismTuning, MechanismTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        assertEquals(0.8, MechanismTuning.kP, 0.0)
        MechanismTuning.kV = 0.06
        assertTrue(ConfigStore.persistIfDirty())

        ConfigStore.reset()
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        assertEquals(0.75, TestTuning.gain, 0.0)
        assertEquals(8, TestTuning.count)
    }

    @Test
    fun persistWithoutLoadStillPreservesUnregisteredKeys() {
        tempFile.writeText("$schemaLine\nlift.kP=0.8\n")
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        TestTuning.gain = 0.75

        assertTrue(ConfigStore.persistIfDirty())
        assertTrue(tempFile.readLines().contains("lift.kP=0.8"))
    }

    @Test
    fun secondInitLoadRestoresNewSectionsWithoutChangingEarlierOverrides() {
        tempFile.writeText("$schemaLine\ntest.gain=0.75\nlift.kP=0.8\n")
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.register("lift", MechanismTuning, MechanismTuning::resetDefaults)
        ConfigStore.loadFromDisk()

        assertEquals(0.75, TestTuning.gain, 0.0)
        assertEquals(0.8, MechanismTuning.kP, 0.0)
        assertFalse(ConfigStore.persistIfDirty())
    }

    @Test
    fun invalidDestinationIsNotDeletedAndSaveCanRetry() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        TestTuning.gain = 0.75
        assertTrue(tempFile.mkdir())

        assertFalse(ConfigStore.persistIfDirty())
        assertTrue("a failed replacement must never delete the destination", tempFile.isDirectory)
        assertFalse(ConfigStore.persistIfDirty())

        assertTrue(tempFile.delete())
        assertTrue(ConfigStore.persistIfDirty())
        TestTuning.gain = 0.0
        ConfigStore.loadFromDisk()
        assertEquals(0.75, TestTuning.gain, 0.0)
        assertFalse(ConfigStore.persistIfDirty())
    }

    @Test
    fun failedTempWritePreservesPreviousFileAndRetries() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        TestTuning.gain = 0.75
        assertTrue(ConfigStore.persistIfDirty())
        val previous = tempFile.readText()
        val blockedTemp = File(tempFile.path + ".tmp")
        assertTrue(blockedTemp.mkdir())
        val marker = File(blockedTemp, "keep").also { it.writeText("occupied") }
        TestTuning.gain = 0.9

        assertFalse(ConfigStore.persistIfDirty())
        assertEquals(previous, tempFile.readText())
        assertTrue(marker.exists())
        assertTrue(blockedTemp.deleteRecursively())
        assertTrue(ConfigStore.persistIfDirty())
        TestTuning.gain = 0.0
        ConfigStore.loadFromDisk()
        assertEquals(0.9, TestTuning.gain, 0.0)
    }

    @Test
    fun failedRenamePreservesPreviousFileAndRetries() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        TestTuning.gain = 0.75
        assertTrue(ConfigStore.persistIfDirty())
        val previous = tempFile.readText()
        TestTuning.gain = 0.9
        var attemptedRenames = 0
        val rejectRename: (File, File) -> Boolean = { _, _ ->
            attemptedRenames++
            false
        }

        assertFalse(ConfigStore.persistIfDirty(rejectRename))
        assertEquals(1, attemptedRenames)
        assertEquals(previous, tempFile.readText())
        assertFalse(ConfigStore.persistIfDirty(rejectRename))
        assertEquals(2, attemptedRenames)
        assertEquals(previous, tempFile.readText())
        assertFalse(File(tempFile.path + ".tmp").exists())
        assertTrue(ConfigStore.persistIfDirty())
        TestTuning.gain = 0.0
        ConfigStore.loadFromDisk()
        assertEquals(0.9, TestTuning.gain, 0.0)
        assertFalse(ConfigStore.persistIfDirty())
    }

    @Test
    fun missingFileLeavesDefaultsUntouched() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        assertEquals(0.5, TestTuning.gain, 0.0)
    }

    @Test
    fun snapshotKeysAreSectionQualified() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        val snapshot = ConfigStore.snapshot()
        assertTrue("test.gain" in snapshot)
        assertTrue("test.enabled" in snapshot)
        assertTrue("test.count" in snapshot)
        assertTrue("test.label" in snapshot)
    }

    @Test
    fun frameworkConfigObjectsExposeTheirTunables() {
        ConfigStore.register("drive", DriveConfig, DriveConfig::resetDefaults)
        ConfigStore.register(
            "localizer",
            LocalizerConfig,
            LocalizerConfig::resetDefaults,
        )
        val snapshot = ConfigStore.snapshot()
        assertTrue("drive.inputExponent" in snapshot)
        assertTrue("drive.fieldCentricDefault" in snapshot)
        assertFalse("drive.fieldCentric" in snapshot)
        assertTrue("localizer.correctionBlend" in snapshot)
        assertTrue("localizer.followingBlendScale" in snapshot)
        // Private defaults and synthetic fields must not leak.
        assertFalse(snapshot.keys.any { "DEFAULT" in it || "INSTANCE" in it })
    }

    @Test
    fun everyProductionTunableIsResetOnWarmInit() {
        val configs = listOf(DriveConfig, LocalizerConfig, BallAimConfig, BallApproachConfig, MotorTestConfig)
        val originals = configs.flatMap { config ->
            config.javaClass.declaredFields
                .filter { Modifier.isPublic(it.modifiers) && !Modifier.isFinal(it.modifiers) }
                .map { field -> Triple(config, field, field.get(config)) }
        }
        ConfigStore.register("drive", DriveConfig, DriveConfig::resetDefaults)
        ConfigStore.register("localizer", LocalizerConfig, LocalizerConfig::resetDefaults)
        ConfigStore.register("ballAim", BallAimConfig, BallAimConfig::resetDefaults)
        ConfigStore.register("ballApproach", BallApproachConfig, BallApproachConfig::resetDefaults)
        ConfigStore.register("motorTest", MotorTestConfig, MotorTestConfig::resetDefaults)
        try {
            ConfigStore.loadFromDisk()
            val defaults = ConfigStore.snapshot()
            for ((config, field) in originals) {
                when (field.type) {
                    java.lang.Double.TYPE -> field.setDouble(config, field.getDouble(config) + 1.0)
                    java.lang.Integer.TYPE -> field.setInt(config, field.getInt(config) + 1)
                    java.lang.Boolean.TYPE -> field.setBoolean(config, !field.getBoolean(config))
                    else -> error("Add a mutation for tunable ${field.name}")
                }
            }

            ConfigStore.loadFromDisk()

            assertEquals(defaults, ConfigStore.snapshot())
        } finally {
            for ((config, field, value) in originals) field.set(config, value)
        }
    }

    @Test
    fun doubleRoundTripIsExact() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        TestTuning.gain = 1.0 / 3.0
        ConfigStore.persistIfDirty()

        TestTuning.gain = 0.0
        ConfigStore.reset()
        ConfigStore.file = tempFile
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        ConfigStore.loadFromDisk()

        assertEquals(1.0 / 3.0, TestTuning.gain, 0.0)
    }

    @Test
    fun schemaMismatchIgnoresStaleSeasonValues() {
        ConfigStore.register("test", TestTuning, TestTuning::resetDefaults)
        TestTuning.gain = 0.8
        tempFile.writeText(
            """
            ${ConfigStore.SCHEMA_KEY}=previous-season
            test.gain=9.0
            lift.kP=8.0
            """.trimIndent(),
        )
        ConfigStore.loadFromDisk()

        assertEquals(0.5, TestTuning.gain, 0.0)
        TestTuning.count = 8
        assertTrue(ConfigStore.persistIfDirty())
        assertFalse(tempFile.readText().contains("lift.kP"))
        assertTrue(tempFile.readLines().contains(schemaLine))
    }

    @Test
    fun primitiveMechanismConfigRoundTripsIntoLiveHolders() {
        ConfigStore.register("lift", MechanismTuning, MechanismTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        MechanismTuning.kP = 0.75
        MechanismTuning.kV = 0.08
        MechanismTuning.maxVelocity = 42.0
        MechanismTuning.maxAcceleration = 84.0
        assertTrue(ConfigStore.persistIfDirty())

        MechanismTuning.kP = 0.1
        MechanismTuning.kV = 0.02
        MechanismTuning.maxVelocity = 30.0
        MechanismTuning.maxAcceleration = 60.0
        ConfigStore.reset()
        ConfigStore.file = tempFile
        ConfigStore.register("lift", MechanismTuning, MechanismTuning::resetDefaults)
        ConfigStore.loadFromDisk()
        MechanismTuning.sync()

        assertEquals(0.75, MechanismTuning.gains.kP, 0.0)
        assertEquals(0.08, MechanismTuning.gains.kV, 0.0)
        assertEquals(42.0, MechanismTuning.constraints.maxVelocity, 0.0)
        assertEquals(84.0, MechanismTuning.constraints.maxAcceleration, 0.0)
    }

    private val schemaLine: String
        get() = "${ConfigStore.SCHEMA_KEY}=${ConfigStore.schemaId}"
}
