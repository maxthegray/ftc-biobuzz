package org.firstinspires.ftc.teamcode.core.logging

import java.io.ByteArrayOutputStream
import org.firstinspires.ftc.teamcode.core.logging.WpiStruct.FIELD_CENTRE_INCHES
import org.firstinspires.ftc.teamcode.core.logging.WpiStruct.POSE2D_SIZE
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

/**
 * The struct wire format AdvantageScope decodes. Every assertion here is a
 * property of *its* decoder rather than a choice of ours, so a "harmless"
 * rename or reorder that breaks the 2D Field tab fails a test instead of
 * silently drawing an empty field.
 */
class WpiStructTest {

    @Test
    fun poseEncodesThreeLittleEndianDoublesInFieldCentredMetres() {
        val buffer = ByteArray(POSE2D_SIZE)
        WpiStruct.encodePose2d(buffer, xInches = 96.0, yInches = 60.0, headingRadians = 1.25)

        val values = doublesOf(buffer)
        assertEquals(3, values.size)
        // 96 in is 24 in past the centre of a 144 in field -> 0.6096 m.
        assertEquals(0.6096, values[0], 1e-12)
        // 60 in is 12 in short of it -> -0.3048 m.
        assertEquals(-0.3048, values[1], 1e-12)
        // Heading is radians on both sides and must pass through untouched.
        assertEquals(1.25, values[2], 0.0)
    }

    @Test
    fun fieldCentreIsTheOrigin() {
        assertEquals(0.0, WpiStruct.toFieldMetres(FIELD_CENTRE_INCHES), 0.0)
    }

    @Test
    fun nonFiniteCoordinatesCollapseToTheCentreInsteadOfPoisoningTheViewer() {
        val buffer = ByteArray(POSE2D_SIZE)
        WpiStruct.encodePose2d(buffer, Double.NaN, Double.POSITIVE_INFINITY, Double.NaN)

        val values = doublesOf(buffer)
        assertArrayEquals(doubleArrayOf(0.0, 0.0, 0.0), values, 0.0)
    }

    @Test
    fun schemasCoverPose2dAndBothNestedTypesWithWpilibsFieldNames() {
        val log = writeLog { WpiStruct.declareSchemas(it) }

        // The decoder keys off this exact entry-name prefix, and resolves the
        // nested types by the names used inside the Pose2d schema.
        assertEquals(
            "double x;double y;",
            schema(log, "/.schema/struct:Translation2d"),
        )
        assertEquals(
            "double value;",
            schema(log, "/.schema/struct:Rotation2d"),
        )
        assertEquals(
            "Translation2d translation;Rotation2d rotation;",
            schema(log, "/.schema/struct:Pose2d"),
        )
        assertEquals("structschema", log.type("/.schema/struct:Pose2d"))
    }

    @Test
    fun nestedSchemasAreDeclaredBeforeTheTypeThatReferencesThem() {
        val log = writeLog { WpiStruct.declareSchemas(it) }
        val ids = log.entries.values.associate { it.name to it.id }

        val pose2d = ids.getValue("/.schema/struct:Pose2d")
        assertEquals(true, ids.getValue("/.schema/struct:Translation2d") < pose2d)
        assertEquals(true, ids.getValue("/.schema/struct:Rotation2d") < pose2d)
    }

    private fun schema(log: WpiLog, name: String): String =
        String(log.raws(name).single().second, Charsets.UTF_8)

    private fun writeLog(block: (WpiLogWriter) -> Unit): WpiLog {
        val bytes = ByteArrayOutputStream()
        WpiLogWriter(bytes).use(block)
        return WpiLog.read(bytes.toByteArray())
    }

    private fun doublesOf(buffer: ByteArray): DoubleArray {
        var bits = 0L
        return DoubleArray(buffer.size / 8) { i ->
            bits = 0L
            for (b in 7 downTo 0) {
                bits = (bits shl 8) or (buffer[i * 8 + b].toLong() and 0xff)
            }
            Double.fromBits(bits)
        }
    }
}
