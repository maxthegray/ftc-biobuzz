package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.CellLocation

/** Which of a HIVE's two CELLs faces up. Each HIVE is one alliance's; the raised CELL is its goal. */
enum class HiveState(val raised: CellLocation) {
    AUDIENCE_RAISED(CellLocation.AUDIENCE),
    FAR_RAISED(CellLocation.FAR);

    val lowered: CellLocation get() = if (this == AUDIENCE_RAISED) CellLocation.FAR else CellLocation.AUDIENCE

    companion object {
        fun withRaised(location: CellLocation): HiveState =
            if (location == CellLocation.AUDIENCE) AUDIENCE_RAISED else FAR_RAISED

        fun withLowered(location: CellLocation): HiveState =
            if (location == CellLocation.AUDIENCE) FAR_RAISED else AUDIENCE_RAISED

        /**
         * Pre-match staging, manual §10.3.1 / Figure 10-2: the CELL that points
         * at a FLOWER is tilted down, leaving RED's audience CELL and BLUE's far
         * CELL up. Confirm on the real field before relying on it in auton.
         */
        fun matchSetup(alliance: Alliance): HiveState =
            if (alliance == Alliance.RED) AUDIENCE_RAISED else FAR_RAISED
    }
}

/** A tag's centre height against the raised/lowered threshold. */
enum class TagHeightClass { RAISED, LOWERED, AMBIGUOUS }

/** A confirmed change of one HIVE's state. */
data class HiveTransition(
    val alliance: Alliance,
    /** Null when the state was unknown until this frame; that is not a tip. */
    val from: HiveState?,
    val to: HiveState,
    val timestampNanos: Long,
    /** The previous state came from the match-setup prior and was never seen. */
    val fromAssumedPrior: Boolean,
    /** Time since the HIVE was last seen in [from] before the contradicting frames, or null. */
    val unobservedMs: Double?,
) {
    val isTip: Boolean get() = from != null

    fun describe(): String {
        val raised = to.raised.name.lowercase()
        if (from == null) return "HIVE STATE: ${alliance.name} $raised CELL raised (first confirmed sighting)"
        val notes = buildList {
            if (fromAssumedPrior) add("previous state assumed from match setup, never seen")
            unobservedMs?.let {
                if (it >= UNOBSERVED_NOTE_MS) add("last seen in the old state %.0f ms earlier".format(java.util.Locale.US, it))
            }
        }
        return "HIVE TIP: ${alliance.name} ${from.raised.name.lowercase()}→$raised" +
            if (notes.isEmpty()) "" else " (${notes.joinToString("; ")})"
    }

    companion object {
        const val UNOBSERVED_NOTE_MS = 250.0
    }
}

/**
 * One HIVE's state from per-frame votes, with hysteresis: a state changes only
 * after [HiveConfig.tipConfirmFrames] consecutive frames vote for the
 * other state, so a single bad solve cannot fake a tip. A frame that votes for
 * the current state cancels a pending change; a frame without a vote (no tags
 * of this HIVE, or tags that disagree) leaves it pending.
 *
 * A tip is only seen when the camera sees this HIVE: one made while the turret
 * looked away is confirmed at the next sightings, and its [lastTipNanos] is
 * the confirming frame's capture time.
 */
class HiveStateEstimator(val alliance: Alliance, prior: HiveState? = null) {

    var state: HiveState? = prior
        private set

    /** True while [state] is the prior and no frame has confirmed it. */
    var assumed: Boolean = prior != null
        private set

    /** Capture time of the newest frame that voted, either way. */
    var lastObservedNanos: Long? = null
        private set

    var tipCount: Int = 0
        private set

    var lastTipNanos: Long? = null
        private set

    private var lastAgreeingNanos: Long? = null
    private var pending: HiveState? = null
    private var pendingFrames = 0
    private var pendingSinceNanos = 0L

    /** Feed one frame's vote; returns the transition when this frame confirms a change. */
    fun observe(vote: HiveState, timestampNanos: Long, confirmFrames: Int): HiveTransition? {
        lastObservedNanos = timestampNanos
        if (vote == state) {
            assumed = false
            lastAgreeingNanos = timestampNanos
            pending = null
            pendingFrames = 0
            return null
        }
        if (vote == pending) {
            pendingFrames++
        } else {
            pending = vote
            pendingFrames = 1
            pendingSinceNanos = timestampNanos
        }
        if (pendingFrames < confirmFrames.coerceAtLeast(1)) return null

        val transition = HiveTransition(
            alliance = alliance,
            from = state,
            to = vote,
            timestampNanos = timestampNanos,
            fromAssumedPrior = assumed,
            unobservedMs = lastAgreeingNanos?.let { (pendingSinceNanos - it) / 1e6 },
        )
        if (transition.isTip) {
            tipCount++
            lastTipNanos = timestampNanos
        }
        state = vote
        assumed = false
        lastAgreeingNanos = timestampNanos
        pending = null
        pendingFrames = 0
        return transition
    }
}
