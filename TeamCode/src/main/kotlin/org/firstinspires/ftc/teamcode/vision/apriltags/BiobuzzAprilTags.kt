package org.firstinspires.ftc.teamcode.vision.apriltags

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
 * Team Update 00 (September 12, 2026) makes no AprilTag change.
 *
 * Deliberately absent: field poses. The HIVES pivot, so a tag's field pose
 * depends on HIVE state; and a sighting says nothing about whether its CELL
 * is upward-facing or ready to score. [HiveCellReadiness] keeps that separate.
 */
object BiobuzzAprilTags {

    const val FAMILY = "36h11"

    /** Printed black-square edge length. */
    const val TAG_SIZE_INCHES = 3.25
    const val TAG_SIZE_MM = TAG_SIZE_INCHES * 25.4

    /** Tag centre offsets along the sticker, from the cluster centreline, slot 1 → 4. */
    private val SLOT_OFFSETS_INCHES = doubleArrayOf(-6.5, -2.75, 2.75, 6.5)

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
 * What these diagnostics are prepared to say about a CELL being ready for
 * scoring. Pass 1 has no evidence that establishes it, so every CELL is
 * [NOT_INFERRED] regardless of which tags are visible.
 */
enum class HiveCellReadiness { NOT_INFERRED }
