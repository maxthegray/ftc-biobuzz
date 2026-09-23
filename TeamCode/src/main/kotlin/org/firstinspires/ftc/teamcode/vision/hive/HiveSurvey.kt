package org.firstinspires.ftc.teamcode.vision.hive

import java.util.Locale
import java.util.TreeMap
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Cell

/**
 * The Hive Tag Survey's memory: the latest row of every season tag and the
 * latest fused goal of every CELL seen this run, so a lab record can list all
 * 16 tags even when they were surveyed from different spots. Times are frame
 * capture times on the robot clock.
 */
class HiveSurvey {

    data class Seen<T>(val value: T, val captureNanos: Long)

    private val tags = TreeMap<Int, Seen<TagRow>>()
    private val cells = LinkedHashMap<Cell, Seen<CellGoal>>()

    fun record(tracker: HiveGoalTracker, captureNanos: Long) {
        for (row in tracker.rows) tags[row.tag.id] = Seen(row, captureNanos)
        for ((cell, goal) in tracker.cellGoals) cells[cell] = Seen(goal, captureNanos)
    }

    fun tag(id: Int): Seen<TagRow>? = tags[id]

    fun cell(cell: Cell): Seen<CellGoal>? = cells[cell]

    fun reset() {
        tags.clear()
        cells.clear()
    }

    /** `[measured]` lines for a lab record: HIVE states, every CELL, every tag. */
    fun measurements(tracker: HiveGoalTracker, mount: TurretCameraMount, nowNanos: Long): List<Pair<String, String>> {
        val m = ArrayList<Pair<String, String>>()
        for ((alliance, estimator) in tracker.estimators) {
            val state = estimator.state?.name ?: "UNKNOWN"
            m += "hive.${alliance.name}" to "$state${if (estimator.assumed) " (assumed)" else ""}; tips ${estimator.tipCount}"
        }
        for (cell in BiobuzzAprilTags.cells) {
            val key = "cell.${cell.stickerLabel.replace(' ', '_')}"
            val seen = cells[cell]
            m += key to (seen?.let { describeCell(it.value, mount) + "; seen ${age(nowNanos, it.captureNanos)} ago" } ?: "not seen")
        }
        for (tag in BiobuzzAprilTags.tags) {
            val seen = tags[tag.id]
            m += "tag.${tag.id}" to (seen?.let { describeTag(it.value) + "; seen ${age(nowNanos, it.captureNanos)} ago" } ?: "${tag.meaning}; not seen")
        }
        return m
    }

    companion object {
        fun describeCell(goal: CellGoal, mount: TurretCameraMount): String =
            "${goal.heightClass} goal ${vec(goal.goalRobot)} in; " +
                "horizontal ${fmt(GoalGeometry.horizontalDistanceFromTurretIn(goal.goalRobot, mount), 1)} in; " +
                "turret bearing ${fmt(Math.toDegrees(GoalGeometry.turretBearingRad(goal.goalRobot, mount)), 1)}°; " +
                "spread ${fmt(goal.spreadIn, 2)} in; tags ${goal.tagIds.joinToString(",")}"

        fun describeTag(row: TagRow): String =
            "${row.tag.meaning}; ${row.heightClass} at ${fmt(row.tagRobot.z, 1)} in; goal ${vec(row.goalRobot)} in; " +
                "off fused ${fmt(row.deviationIn, 2)} in${if (row.usedInFusion) "" else " (dropped)"}; " +
                "faces ${fmt(row.facingBelowHorizontalDeg, 0)}° below horizontal"

        private fun vec(v: Vec3) = "(${fmt(v.x, 1)}, ${fmt(v.y, 1)}, ${fmt(v.z, 1)})"

        private fun age(nowNanos: Long, thenNanos: Long) = "${fmt((nowNanos - thenNanos) / 1e9, 1)} s"

        fun fmt(value: Double, decimals: Int): String =
            if (value.isFinite()) "%.${decimals}f".format(Locale.US, value) else value.toString()
    }
}
