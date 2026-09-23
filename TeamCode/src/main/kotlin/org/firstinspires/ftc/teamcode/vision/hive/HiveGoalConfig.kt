package org.firstinspires.ftc.teamcode.vision.hive

import com.bylazar.configurables.annotations.Configurable

/**
 * HIVE state and goal-fusion settings, persisted under `hiveGoal`. Goal
 * geometry itself is FIRST's (see `BiobuzzAprilTags`) and is not tunable.
 *
 * [raisedMinHeightIn] defaults to the pivot height (manual §9.6.1, 43.95 in.):
 * raised-CELL tags sit above it and lowered-CELL tags below. Set it from the
 * Hive Tag Survey to the midpoint of the heights it reports for both states.
 */
@Configurable
object HiveGoalConfig {

    private const val DEFAULT_RAISED_MIN_HEIGHT_IN = 43.95
    private const val DEFAULT_CLASSIFICATION_MARGIN_IN = 3.0
    private const val DEFAULT_MAX_TAG_SPREAD_IN = 2.0
    private const val DEFAULT_LOST_TIMEOUT_MS = 500
    private const val DEFAULT_TIP_CONFIRM_FRAMES = 3

    /** Tag-centre height above the tiles separating a raised CELL's tags from a lowered CELL's. */
    @JvmField var raisedMinHeightIn: Double = DEFAULT_RAISED_MIN_HEIGHT_IN

    /** Tags within this of [raisedMinHeightIn] are ambiguous (e.g. mid-tip) and cast no HIVE-state vote. */
    @JvmField var classificationMarginIn: Double = DEFAULT_CLASSIFICATION_MARGIN_IN

    /** Tags whose goal point is farther than this from the cluster's median are dropped from the fused goal. */
    @JvmField var maxTagSpreadIn: Double = DEFAULT_MAX_TAG_SPREAD_IN

    /** A goal not re-observed for this long is reported lost. */
    @JvmField var lostTimeoutMs: Int = DEFAULT_LOST_TIMEOUT_MS

    /** Consecutive agreeing frames required to change a HIVE's state. */
    @JvmField var tipConfirmFrames: Int = DEFAULT_TIP_CONFIRM_FRAMES

    fun resetDefaults() {
        raisedMinHeightIn = DEFAULT_RAISED_MIN_HEIGHT_IN
        classificationMarginIn = DEFAULT_CLASSIFICATION_MARGIN_IN
        maxTagSpreadIn = DEFAULT_MAX_TAG_SPREAD_IN
        lostTimeoutMs = DEFAULT_LOST_TIMEOUT_MS
        tipConfirmFrames = DEFAULT_TIP_CONFIRM_FRAMES
    }

    fun compiledDefaults(): Map<String, Any> = linkedMapOf(
        "raisedMinHeightIn" to DEFAULT_RAISED_MIN_HEIGHT_IN,
        "classificationMarginIn" to DEFAULT_CLASSIFICATION_MARGIN_IN,
        "maxTagSpreadIn" to DEFAULT_MAX_TAG_SPREAD_IN,
        "lostTimeoutMs" to DEFAULT_LOST_TIMEOUT_MS,
        "tipConfirmFrames" to DEFAULT_TIP_CONFIRM_FRAMES,
    )

    /** Immutable, validated copy for one tick. */
    fun snapshot() = HiveGoalSettings(
        raisedMinHeightIn = raisedMinHeightIn.finiteOr(DEFAULT_RAISED_MIN_HEIGHT_IN),
        classificationMarginIn = classificationMarginIn.takeIf { it.isFinite() && it >= 0.0 } ?: DEFAULT_CLASSIFICATION_MARGIN_IN,
        maxTagSpreadIn = maxTagSpreadIn.takeIf { it.isFinite() && it > 0.0 } ?: DEFAULT_MAX_TAG_SPREAD_IN,
        lostTimeoutMs = (if (lostTimeoutMs > 0) lostTimeoutMs else DEFAULT_LOST_TIMEOUT_MS).toLong(),
        tipConfirmFrames = if (tipConfirmFrames >= 1) tipConfirmFrames else DEFAULT_TIP_CONFIRM_FRAMES,
    )
}

data class HiveGoalSettings(
    val raisedMinHeightIn: Double = 43.95,
    val classificationMarginIn: Double = 3.0,
    val maxTagSpreadIn: Double = 2.0,
    val lostTimeoutMs: Long = 500L,
    val tipConfirmFrames: Int = 3,
)
