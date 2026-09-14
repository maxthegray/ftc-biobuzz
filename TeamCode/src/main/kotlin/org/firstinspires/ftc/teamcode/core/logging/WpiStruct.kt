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
 * [encodePose2d] takes a pose in Pedro's frame and writes it in the frame
 * AdvantageScope's *Center/Rotated* FTC coordinate system draws. The two
 * differ in three ways, all applied here so the struct is self-describing:
 *
 *  - **Metres, not inches.** AdvantageScope's internal geometry is metric
 *    (`Translation2d = [number, number] // meters`).
 *  - **Field-centre origin, not a corner.** The FTC frame puts (0, 0) at the
 *    middle of the field; Pedro's origin is a corner, so poses run 0..144
 *    inches. [FIELD_CENTRE_INCHES] is that offset.
 *  - **A quarter-turn rotation that depends on the season.** Pedro's frame is
 *    fixed by the audience: (0, 0) is the corner on the audience's left, +X
 *    runs along the audience wall to the right, +Y runs away from the
 *    audience. The FTC frame is fixed by the red alliance wall (+Y points from
 *    the red wall to the blue wall) and the audience, and which wall is red
 *    changes with the game. `quarterTurns` is the counter-clockwise quarter
 *    turns from Pedro's axes onto the FTC axes; the season's value lives in
 *    `RobotConfig.Field.FIELD_VIEW_QUARTER_TURNS`. Heading turns with the axes.
 *
 * This rotation is *display only*. `pose`, `Constants.java`, autonomous start
 * poses and `Alliance` all stay in Pedro's frame and never see it.
 */
object WpiStruct {

    /** WPILOG entry type for a channel holding [encodePose2d] payloads. */
    const val POSE2D_TYPE = "struct:Pose2d"

    /** Bytes one [encodePose2d] payload occupies: three little-endian doubles. */
    const val POSE2D_SIZE = 24

    /** Half of the 12-foot FTC field, in inches — the corner-to-centre offset. */
    const val FIELD_CENTRE_INCHES = 72.0

    private const val INCHES_TO_METRES = 0.0254
    private const val QUARTER_TURN = Math.PI / 2
    private const val FULL_TURN = 2 * Math.PI

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
     * Encodes a Pedro-frame pose into [buffer] as a WPILib `Pose2d` in the FTC
     * frame: `Translation2d(x, y)` then `Rotation2d(value)`, three
     * little-endian doubles. The pose is moved to the field centre, converted
     * to metres and rotated [quarterTurns] quarter turns counter-clockwise
     * (see the class note); the heading is rotated the same way and kept in
     * Pedro's [0, 2π). A non-finite coordinate lands on the centre line it
     * feeds and a non-finite heading becomes 0, rather than poisoning the
     * viewer's auto-scaling. [buffer] must hold at least [POSE2D_SIZE] bytes
     * and is reused across ticks — this runs on the hot loop.
     */
    fun encodePose2d(
        buffer: ByteArray,
        xInches: Double,
        yInches: Double,
        headingRadians: Double,
        quarterTurns: Int,
    ) {
        val x = toFieldMetres(xInches)
        val y = toFieldMetres(yInches)
        val fieldX: Double
        val fieldY: Double
        when (Math.floorMod(quarterTurns, 4)) {
            0 -> { fieldX = x; fieldY = y }
            1 -> { fieldX = -y; fieldY = x }
            2 -> { fieldX = -x; fieldY = -y }
            else -> { fieldX = y; fieldY = -x }
        }
        putDouble(buffer, 0, fieldX + 0.0) // + 0.0 turns a -0.0 centre into 0.0
        putDouble(buffer, 8, fieldY + 0.0)
        putDouble(buffer, 16, toFieldHeading(headingRadians, quarterTurns))
    }

    /**
     * Pedro inches measured from a field corner to metres measured from the
     * field centre, along the same axis. A non-finite coordinate becomes the
     * field centre.
     */
    fun toFieldMetres(inches: Double): Double =
        if (inches.isFinite()) (inches - FIELD_CENTRE_INCHES) * INCHES_TO_METRES else 0.0

    /**
     * Pedro heading rotated with the axes by [quarterTurns], normalised to
     * [0, 2π) like Pedro's own headings. Non-finite becomes 0.
     */
    fun toFieldHeading(headingRadians: Double, quarterTurns: Int): Double {
        if (!headingRadians.isFinite()) return 0.0
        val turned = headingRadians + Math.floorMod(quarterTurns, 4) * QUARTER_TURN
        val wrapped = turned % FULL_TURN
        return if (wrapped < 0) wrapped + FULL_TURN else wrapped
    }

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
