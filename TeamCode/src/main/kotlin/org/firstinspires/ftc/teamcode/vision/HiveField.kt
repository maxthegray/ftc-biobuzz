package org.firstinspires.ftc.teamcode.vision

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.math.Pose
import kotlin.math.cos
import kotlin.math.sin
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.Cell
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.CellLocation

// The HIVE's fixed facts in one place: the field tags (BiobuzzAprilTags), the
// approximate goal points (HiveField), and the live-tunable settings (HiveConfig).

/**
 * The 16 BIOBUZZ field AprilTags: identity and printed geometry only.
 *
 * Source: *2026-2027 FIRST Tech Challenge Competition Manual, BIOBUZZ*, V1
 * (September 12, 2026), https://ftc-resources.firstinspires.org/ftc/game/manual
 *  - §9.6 / §9.6.2: each HIVE pivots between two stable positions, one CELL
 *    facing upward at a time; "On the bottom face of each CELL is a unique
 *    AprilTag Cluster containing 4 distinct AprilTags."
 *  - §9.9: 3.25 in. (8.25 cm) square targets, tag family 36h11; four tags per
 *    cluster sticker, each with an ID label.
 *  - Figure 9-15: tag centres at ±2.75 in. and ±6.5 in. from the cluster
 *    centreline; reference holes at ±7.0 in.; tag centres 2.75 in. above the
 *    reference-hole centreline. The sticker prints IDs in ascending order left
 *    to right with its coloured label band at the bottom (e.g. ID 30 … ID 33 on
 *    the "RED SCORING" sticker).
 *  - Figure 9-16 / page 76: cluster faces the TILES with its bottom edge towards
 *    the field centre; IDs 30–33 red CELL opposite the audience, 34–37 red CELL
 *    audience side, 38–41 blue CELL audience side, 42–45 blue CELL opposite the
 *    audience. Figure 9-17 labels the opposite-audience clusters "Scoring Tags".
 * Team Updates 00–02 (through September 24, 2026) make no AprilTag change.
 *
 * Cluster geometry relative to the goal comes from FTC SDK 12.0.0
 * `AprilTagGameDatabase.getBioBuzzCluster`: the cluster origin is the centre
 * of the CELL opening, and every tag centre sits at
 * ([Tag.offsetFromClusterCenterInches], [TAG_ROW_Y_INCHES], [TAG_ROW_Z_INCHES])
 * in the cluster frame: +X along the sticker in slot order, +Y towards the
 * label band, +Z into the sticker face. 7.1874 in. is Figure 9-15's 9.938 in.
 * less 2.75 in.; 5.622 in. is the centroid of the Figure 9-11 opening.
 *
 * Deliberately absent: field poses. The HIVES pivot, so a tag's field pose
 * depends on HIVE state, which `HiveTracker` infers from measured tag heights.
 * A single sighting says nothing about whether its CELL is raised.
 * `HiveField` holds approximate raised-CELL goal points, for aiming only.
 */
object BiobuzzAprilTags {

    const val FAMILY = "36h11"

    /** Printed black-square edge length. */
    const val TAG_SIZE_INCHES = 3.25
    const val TAG_SIZE_MM = TAG_SIZE_INCHES * 25.4

    /** Tag centre offsets along the sticker, from the cluster centreline, slot 1 → 4. */
    private val SLOT_OFFSETS_INCHES = doubleArrayOf(-6.5, -2.75, 2.75, 6.5)

    /** Tag centres from the CELL opening centre, towards the label band (cluster +Y). */
    const val TAG_ROW_Y_INCHES = 7.1874

    /** Tag centres from the CELL opening centre, into the sticker face (cluster +Z); negative: the opening is behind the face. */
    const val TAG_ROW_Z_INCHES = -5.622

    enum class Alliance { RED, BLUE }

    enum class CellLocation(val stickerWord: String, val description: String) {
        AUDIENCE("AUDIENCE", "audience side"),
        FAR("SCORING", "side opposite the audience"),
    }

    data class Cell(val alliance: Alliance, val location: CellLocation) {
        /** The text printed on the sticker label band, e.g. "RED SCORING". */
        val stickerLabel: String get() = "${alliance.name} ${location.stickerWord}"

        override fun toString(): String = "${alliance.name} CELL, ${location.description}"
    }

    data class Tag(
        val id: Int,
        val cell: Cell,
        /** 1–4, left to right on the printed sticker read with its label band at the bottom. */
        val slot: Int,
        /** Tag centre from the cluster centreline along the sticker, same left-to-right sense as [slot]. */
        val offsetFromClusterCenterInches: Double,
    ) {
        /**
         * The CELL opening centre in this tag's frame (+X right looking at the
         * tag, +Y towards the label band, +Z into the face), inches. Each tag
         * of a cluster points at the same goal from a different offset.
         */
        val goalXInTagInches: Double get() = -offsetFromClusterCenterInches
        val goalYInTagInches: Double get() = -TAG_ROW_Y_INCHES
        val goalZInTagInches: Double get() = -TAG_ROW_Z_INCHES

        val meaning: String get() = "${cell.stickerLabel} slot $slot"
    }

    val cells: List<Cell> = listOf(
        Cell(Alliance.RED, CellLocation.FAR),
        Cell(Alliance.RED, CellLocation.AUDIENCE),
        Cell(Alliance.BLUE, CellLocation.AUDIENCE),
        Cell(Alliance.BLUE, CellLocation.FAR),
    )

    private val FIRST_IDS = intArrayOf(30, 34, 38, 42)

    val tags: List<Tag> = cells.flatMapIndexed { cellIndex, cell ->
        (0 until 4).map { slotIndex ->
            Tag(
                id = FIRST_IDS[cellIndex] + slotIndex,
                cell = cell,
                slot = slotIndex + 1,
                offsetFromClusterCenterInches = SLOT_OFFSETS_INCHES[slotIndex],
            )
        }
    }

    private val byId: Map<Int, Tag> = tags.associateBy { it.id }

    fun lookup(id: Int): Tag? = byId[id]

    fun tagsOf(cell: Cell): List<Tag> = tags.filter { it.cell == cell }

    /** Limelight reports families like "36H11C"; compare on the family digits only. */
    fun isSeasonFamily(reported: String): Boolean =
        reported.lowercase().filter { it.isLetterOrDigit() }.removeSuffix("c").endsWith(FAMILY)
}

/**
 * Approximate field position of each raised CELL's opening, for aiming before
 * the Limelight has seen the goal. An aim hint only: never used for
 * localization, and never a reason to shoot.
 *
 * Source: *2026-2027 FIRST Tech Challenge Competition Manual, BIOBUZZ*, V1
 * (September 12, 2026), §9.6, unchanged for these figures through Team Update
 * 02 (September 24, 2026). No coordinates are published, so these are
 * derived from the drawings (manual tolerance ±1 in.):
 *  - §9.6: the HIVE structure is at the centre of the FIELD.
 *  - Figure 9-10: HIVE centre to centre 25.5 in.; Figure 9-17: the red HIVE is
 *    on the audience's left, each HIVE's CELLs on the audience and far sides.
 *  - §9.6.1: pivot axis 43.95 in. above the TILES.
 *  - Figure 9-9: CELLs 18.84 in. apart and 12.04 in. deep along the arm, so a
 *    CELL's outer end face is 21.46 in. from the pivot.
 *  - Figure 9-10: a raised CELL is tilted 30°; its opening (the outer end face)
 *    spans 53.5 to 65.6 in. above the TILES.
 * The opening centre is then 59.55 in. high; solving pivot + 21.46 sin 30° +
 * h cos 30° = 59.55 for the face's offset h puts it 21.46 cos 30° − h sin 30°
 * = 15.8 in. from the pivot, horizontally, towards its own side.
 *
 * Pedro field frame: origin at the corner on the audience's left, +X along the
 * audience wall, +Y away from the audience, inches. Robot frame as in
 * [GoalGeometry]: x forward, y left.
 */
object HiveField {

    const val CENTRE_IN = RobotConfig.Field.LENGTH_INCHES / 2.0
    const val HIVE_CENTRE_TO_CENTRE_IN = 25.5
    const val RAISED_OPENING_FROM_PIVOT_IN = 15.8
    const val RAISED_OPENING_HEIGHT_IN = 59.55

    /** Pivot x of [alliance]'s HIVE. */
    fun pivotX(alliance: Alliance): Double =
        if (alliance == Alliance.RED) CENTRE_IN - HIVE_CENTRE_TO_CENTRE_IN / 2.0 else CENTRE_IN + HIVE_CENTRE_TO_CENTRE_IN / 2.0

    /** Opening centre of [cell] while it is the raised CELL, field inches. */
    fun goal(cell: Cell): Vec3 {
        val towardFar = if (cell.location == CellLocation.FAR) 1.0 else -1.0
        return Vec3(pivotX(cell.alliance), CENTRE_IN + towardFar * RAISED_OPENING_FROM_PIVOT_IN, RAISED_OPENING_HEIGHT_IN)
    }

    /** The CELL on the robot's half of the field. */
    fun cellOnSide(robotY: Double): CellLocation = if (robotY < CENTRE_IN) CellLocation.AUDIENCE else CellLocation.FAR

    /** A field point in the frame of a robot at [pose]; z is unchanged. */
    fun toRobot(field: Vec3, pose: Pose): Vec3 {
        val dx = field.x - pose.x()
        val dy = field.y - pose.y()
        val c = cos(pose.heading())
        val s = sin(pose.heading())
        return Vec3(dx * c + dy * s, -dx * s + dy * c, field.z)
    }

    /** A robot-frame point on the field for a robot at [pose]; z is unchanged. */
    fun toField(robot: Vec3, pose: Pose): Vec3 {
        val c = cos(pose.heading())
        val s = sin(pose.heading())
        return Vec3(pose.x() + robot.x * c - robot.y * s, pose.y() + robot.x * s + robot.y * c, robot.z)
    }
}

/**
 * HIVE tracking settings, live-editable in Panels: where the Limelight sits on
 * the turret, and the thresholds for HIVE state and goal fusion. Goal geometry
 * itself is FIRST's (see `BiobuzzAprilTags`) and is not tunable.
 *
 * Robot frame: x forward, y left, z up from the tiles, origin at the robot's
 * pose point. Turret angle 0 points the turret at the robot's front, positive
 * counterclockwise. For a bench survey the camera fields describe a tripod
 * with the turret angle held at 0. The Limelight must be mounted upright
 * (image not flipped).
 *
 * Goal height, and with it raised/lowered HIVE state, is only as good as
 * [cameraHeightIn] and [cameraPitchUpDeg]. [raisedMinHeightIn] defaults to the
 * pivot height (manual §9.6.1, 43.95 in.); set it from the Hive Tag Survey to
 * the midpoint of the tag heights it reports for both states.
 */
@Configurable
object HiveConfig {

    /** Turret rotation axis from the robot's pose point: + toward the robot's front, inches. */
    @JvmField var axisForwardIn = 0.0

    /** Turret rotation axis from the robot's pose point: + toward the robot's left, inches. */
    @JvmField var axisLeftIn = 0.0

    /** Lens from the turret axis at turret angle 0: + toward the robot's front, inches. */
    @JvmField var cameraForwardIn = 0.0

    /** Lens from the turret axis at turret angle 0: + toward the robot's left, inches. */
    @JvmField var cameraLeftIn = 0.0

    /** Lens centre above the tiles, inches. */
    @JvmField var cameraHeightIn = 0.0

    /** Optical axis above horizontal, degrees; positive tilts the camera up toward the tags. */
    @JvmField var cameraPitchUpDeg = 0.0

    /** Optical axis from the turret's forward direction, degrees, counterclockwise-positive. */
    @JvmField var cameraYawDeg = 0.0

    /** True once the camera fields above come from a tape measure. */
    @JvmField var mountMeasured = false

    /** Tag-centre height above the tiles separating a raised CELL's tags from a lowered CELL's. */
    @JvmField var raisedMinHeightIn = 43.95

    /** Tags within this of [raisedMinHeightIn] are ambiguous (e.g. mid-tip) and cast no HIVE-state vote. */
    @JvmField var classificationMarginIn = 3.0

    /** Tags whose goal point is farther than this from the CELL's median are dropped from the fused goal. */
    @JvmField var maxTagSpreadIn = 2.0

    /** A goal not re-observed for this long is reported lost. */
    @JvmField var lostTimeoutMs = 500

    /** Consecutive agreeing frames required to change a HIVE's state. */
    @JvmField var tipConfirmFrames = 3
}
