package org.firstinspires.ftc.teamcode.core.logging

/**
 * WPILib struct encoding for the geometry AdvantageScope's 2D Field tab draws.
 *
 * AdvantageScope renders a robot only from a byte-encoded struct; the older
 * `double[]` "number array" geometry format is deprecated and is removed in
 * 2027. A struct channel carries its own layout, so no per-field configuration
 * is needed in the viewer — but the layout has to come from the log. Its
 * decoder builds `<channel>/translation/x`, `<channel>/translation/y`, and
 * `<channel>/rotation/value` from the schema entries [declareSchemas] writes,
 * then reads exactly those three keys. Nothing is built in; without the schema
 * records the channel decodes as opaque bytes and the field stays empty.
 *
 * Two conversions are baked in because the struct is unit-safe by contract and
 * ours is not:
 *
 *  - **Metres, not inches.** AdvantageScope's internal geometry is metric
 *    (`Translation2d = [number, number] // meters`).
 *  - **Field-centre origin, not a corner.** AdvantageScope's FTC frames put
 *    (0, 0) at the middle of a 12-foot field; Pedro's origin is a corner, so
 *    poses run 0..144 inches. [FIELD_CENTRE_INCHES] is that offset.
 *
 * Heading passes through untouched: both sides are radians, CCW-positive.
 *
 * What this does *not* do is rotate the axes. Pedro's +X and the FTC frame's
 * +X are the same ray only if the field origin was set up that way, which is a
 * team convention rather than something the code can know — see the
 * verification note in `OPERATIONS.md`. If the plotted robot comes out rotated
 * a quarter turn, that is the axis convention, not this encoding.
 */
object WpiStruct {

    /** WPILOG entry type for a channel holding [encodePose2d] payloads. */
    const val POSE2D_TYPE = "struct:Pose2d"

    /** Bytes one [encodePose2d] payload occupies: three little-endian doubles. */
    const val POSE2D_SIZE = 24

    /** Half of the 12-foot FTC field, in inches — the corner-to-centre offset. */
    const val FIELD_CENTRE_INCHES = 72.0

    private const val INCHES_TO_METRES = 0.0254

    /**
     * Registers the schemas the decoder needs, once per log. `Pose2d` is
     * defined in terms of the two nested types, so all three must be present.
     * Entry names are the `/.schema/struct:<type>` convention the decoder
     * matches on; the type string is unrecognised by the reader and so lands
     * in its raw path, which is where schema registration happens.
     */
    fun declareSchemas(writer: WpiLogWriter) {
        for ((type, schema) in SCHEMAS) {
            val entry = writer.startEntry("/.schema/struct:$type", "structschema")
            val bytes = schema.toByteArray(Charsets.UTF_8)
            writer.appendRaw(entry, bytes, timestampUs = 0L)
        }
    }

    /**
     * Encodes a Pedro-frame pose into [buffer] as a WPILib `Pose2d`:
     * `Translation2d(x, y)` then `Rotation2d(value)`, three little-endian
     * doubles. [buffer] must hold at least [POSE2D_SIZE] bytes and is reused
     * across ticks — this runs on the hot loop.
     */
    fun encodePose2d(buffer: ByteArray, xInches: Double, yInches: Double, headingRadians: Double) {
        putDouble(buffer, 0, toFieldMetres(xInches))
        putDouble(buffer, 8, toFieldMetres(yInches))
        putDouble(buffer, 16, if (headingRadians.isFinite()) headingRadians else 0.0)
    }

    /**
     * Pedro inches measured from a field corner to metres measured from the
     * field centre. A non-finite coordinate becomes the field centre rather
     * than poisoning the viewer's auto-scaling.
     */
    fun toFieldMetres(inches: Double): Double =
        if (inches.isFinite()) (inches - FIELD_CENTRE_INCHES) * INCHES_TO_METRES else 0.0

    private fun putDouble(buffer: ByteArray, offset: Int, value: Double) {
        val bits = java.lang.Double.doubleToRawLongBits(value)
        for (i in 0 until 8) {
            buffer[offset + i] = ((bits ushr (8 * i)) and 0xff).toByte()
        }
    }

    /**
     * Type name to schema string, in dependency order. The strings are the
     * WPILib definitions verbatim — the decoder resolves `Translation2d` and
     * `Rotation2d` by name, so neither the field names nor the order may
     * change.
     */
    private val SCHEMAS = listOf(
        "Translation2d" to "double x;double y;",
        "Rotation2d" to "double value;",
        "Pose2d" to "Translation2d translation;Rotation2d rotation;",
    )
}
