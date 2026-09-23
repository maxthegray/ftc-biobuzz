package org.firstinspires.ftc.teamcode.vision.hive

import com.pedropathing.math.Pose
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.CellLocation
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.cell
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.cluster
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.lowered
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.mount
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.raised
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class HiveGoalTrackerTest {

    private val ms = 1_000_000L
    private val settings = HiveGoalSettings(tipConfirmFrames = 2)
    private val matchSetup = mapOf(
        Alliance.RED to HiveState.matchSetup(Alliance.RED),
        Alliance.BLUE to HiveState.matchSetup(Alliance.BLUE),
    )

    @Test
    fun fourTagsOfTheRaisedCellFuseIntoItsOpeningCentre() {
        val tracker = HiveGoalTracker(matchSetup)
        tracker.update(TagFrame(raised(Alliance.RED, CellLocation.AUDIENCE, 90.0, 12.0), 0, 0.0), mount, settings)

        val goal = tracker.goal(Alliance.RED, 5 * ms, null, mount, settings)!!
        assertEquals(cell(Alliance.RED, CellLocation.AUDIENCE), goal.cell)
        assertEquals(listOf(34, 35, 36, 37), goal.tagIds)
        assertEquals(90.0, goal.goalRobot.x, 1e-6)
        assertEquals(12.0, goal.goalRobot.y, 1e-6)
        assertEquals(HiveTestFrames.RAISED_GOAL_HEIGHT_IN, goal.heightIn, 1e-6)
        assertEquals(Math.hypot(90.0, 12.0), goal.horizontalDistanceIn, 1e-6)
        assertEquals(Math.atan2(12.0, 90.0), goal.turretBearingRad, 1e-9)
        assertEquals(0.0, goal.spreadIn, 1e-6)
        assertEquals(5.0, goal.ageMs, 1e-9)
        assertFalse(goal.reprojected)

        assertTrue(tracker.rows.all { it.heightClass == TagHeightClass.RAISED && it.usedInFusion })
        assertEquals(HiveTestFrames.RAISED_GOAL_HEIGHT_IN - 7.1874, tracker.rows.first().tagRobot.z, 1e-6)
    }

    @Test
    fun anOutlierTagIsDroppedFromTheFusedGoal() {
        val tracker = HiveGoalTracker(matchSetup)
        val frame = cluster(
            cell(Alliance.RED, CellLocation.AUDIENCE),
            Vec3(80.0, 0.0, 60.0),
            perturbIn = mapOf(36 to Vec3(10.0, 0.0, 0.0)),
        )
        tracker.update(TagFrame(frame, 0, 0.0), mount, settings)

        val goal = tracker.goal(Alliance.RED, 0, null, mount, settings)!!
        assertEquals(listOf(34, 35, 37), goal.tagIds)
        assertEquals(0.0, goal.goalRobot.y, 1e-6)
        val outlier = tracker.rows.single { it.tag.id == 36 }
        assertFalse(outlier.usedInFusion)
        assertEquals(10.0, outlier.deviationIn, 1e-6)
    }

    @Test
    fun loweredCellTagsAloneRevealTheRaisedCell() {
        val tracker = HiveGoalTracker()
        repeat(2) { i ->
            tracker.update(TagFrame(lowered(Alliance.BLUE, CellLocation.AUDIENCE), i * 10 * ms, 0.0), mount, settings)
        }
        assertEquals(HiveState.FAR_RAISED, tracker.estimators.getValue(Alliance.BLUE).state)
        assertNull("the goal CELL itself was not seen", tracker.goal(Alliance.BLUE, 20 * ms, null, mount, settings))
    }

    @Test
    fun theGoalFollowsTheRaisedCellAndNotTheLoweredOne() {
        val tracker = HiveGoalTracker(matchSetup)
        val frame = raised(Alliance.RED, CellLocation.AUDIENCE, 70.0, 10.0) +
            lowered(Alliance.RED, CellLocation.FAR, 70.0, -10.0)
        tracker.update(TagFrame(frame, 0, 0.0), mount, settings)

        val goal = tracker.goal(Alliance.RED, 0, null, mount, settings)!!
        assertEquals(CellLocation.AUDIENCE, goal.cell.location)
        assertEquals(10.0, goal.goalRobot.y, 1e-6)
        assertEquals(2, tracker.cellGoals.size)
    }

    @Test
    fun aConfirmedTipMovesTheGoalToTheOtherCell() {
        val tracker = HiveGoalTracker(matchSetup)
        tracker.update(TagFrame(raised(Alliance.RED, CellLocation.AUDIENCE), 0, 0.0), mount, settings)

        val tipped = lowered(Alliance.RED, CellLocation.AUDIENCE, 70.0, 10.0) + raised(Alliance.RED, CellLocation.FAR, 70.0, -10.0)
        assertTrue(tracker.update(TagFrame(tipped, 10 * ms, 0.0), mount, settings).isEmpty())
        assertEquals("not yet confirmed; audience goal is held", CellLocation.AUDIENCE, tracker.goal(Alliance.RED, 10 * ms, null, mount, settings)!!.cell.location)

        val transitions = tracker.update(TagFrame(tipped, 20 * ms, 0.0), mount, settings)
        assertEquals(1, transitions.size)
        assertTrue(transitions.single().isTip)
        val goal = tracker.goal(Alliance.RED, 20 * ms, null, mount, settings)!!
        assertEquals(CellLocation.FAR, goal.cell.location)
        assertEquals(-10.0, goal.goalRobot.y, 1e-6)
        assertEquals(1, tracker.estimators.getValue(Alliance.RED).tipCount)
    }

    @Test
    fun tagsNearTheThresholdCastNoVote() {
        val tracker = HiveGoalTracker(matchSetup)
        val nearThreshold = cluster(cell(Alliance.RED, CellLocation.FAR), Vec3(80.0, 0.0, 43.95 + 7.1874))
        repeat(5) { i -> tracker.update(TagFrame(nearThreshold, i * 10 * ms, 0.0), mount, settings) }

        assertTrue(tracker.rows.all { it.heightClass == TagHeightClass.AMBIGUOUS })
        assertEquals(HiveState.AUDIENCE_RAISED, tracker.estimators.getValue(Alliance.RED).state)
        assertTrue(tracker.estimators.getValue(Alliance.RED).assumed)
    }

    @Test
    fun goalsExpireAfterTheLostTimeout() {
        val tracker = HiveGoalTracker(matchSetup)
        tracker.update(TagFrame(raised(Alliance.RED, CellLocation.AUDIENCE), 0, 0.0), mount, settings)
        assertNotNull(tracker.goal(Alliance.RED, settings.lostTimeoutMs * ms, null, mount, settings))
        assertNull(tracker.goal(Alliance.RED, settings.lostTimeoutMs * ms + 1, null, mount, settings))
    }

    @Test
    fun aHeldGoalIsReprojectedOntoTheCurrentPose() {
        val tracker = HiveGoalTracker(matchSetup)
        tracker.update(TagFrame(raised(Alliance.RED, CellLocation.AUDIENCE, 50.0, 0.0), 0, 0.0, Pose(10.0, 20.0, 0.0)), mount, settings)

        val goal = tracker.goal(Alliance.RED, 50 * ms, Pose(10.0, 20.0, PI / 2), mount, settings)!!
        assertTrue(goal.reprojected)
        assertEquals(0.0, goal.goalRobot.x, 1e-6)
        assertEquals(-50.0, goal.goalRobot.y, 1e-6)
        assertEquals(-PI / 2, goal.robotBearingRad, 1e-9)
        assertEquals(HiveTestFrames.RAISED_GOAL_HEIGHT_IN, goal.heightIn, 1e-6)
    }

    @Test
    fun bothAlliancesAreTrackedFromOneFrame() {
        val tracker = HiveGoalTracker(matchSetup)
        val frame = raised(Alliance.RED, CellLocation.AUDIENCE, 80.0, 15.0) + raised(Alliance.BLUE, CellLocation.FAR, 80.0, -15.0)
        tracker.update(TagFrame(frame, 0, 0.0), mount, settings)

        assertEquals(15.0, tracker.goal(Alliance.RED, 0, null, mount, settings)!!.goalRobot.y, 1e-6)
        assertEquals(-15.0, tracker.goal(Alliance.BLUE, 0, null, mount, settings)!!.goalRobot.y, 1e-6)
        assertFalse(tracker.estimators.getValue(Alliance.RED).assumed)
        assertFalse(tracker.estimators.getValue(Alliance.BLUE).assumed)
    }

    @Test
    fun nonSeasonTagsAndUnsolvedPosesAreIgnored() {
        val tracker = HiveGoalTracker(matchSetup)
        val stray = listOf(
            HiveTestFrames.fiducial(12, Vec3(0.0, -50.0, 80.0)),
            HiveTestFrames.fiducial(34, Vec3(0.0, -50.0, 80.0), family = "16H5C"),
            HiveTestFrames.fiducial(35, Vec3(0.0, -50.0, 80.0)).copy(targetPoseCameraSpace = null),
        )
        tracker.update(TagFrame(stray, 0, 0.0), mount, settings)
        assertTrue(tracker.rows.isEmpty())
        assertNull(tracker.goal(Alliance.RED, 0, null, mount, settings))
    }
}
