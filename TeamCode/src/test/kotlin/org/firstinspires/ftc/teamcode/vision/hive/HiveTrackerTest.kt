package org.firstinspires.ftc.teamcode.vision.hive

import com.pedropathing.math.Pose
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.CellLocation
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.cell
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.cluster
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.fiducial
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.lowered
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.raised
import org.junit.After
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class HiveTrackerTest {

    @Before
    fun setUp() {
        HiveConfig.resetDefaults()
        HiveConfig.tipConfirmFrames = 2
    }

    @After
    fun tearDown() = HiveConfig.resetDefaults()

    @Test
    fun fourTagsOfTheRaisedCellFuseIntoItsOpeningCentre() {
        val rig = HiveRig()
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE, 90.0, 12.0))

        val goal = rig.tracker.goal(Alliance.RED)!!
        assertEquals(cell(Alliance.RED, CellLocation.AUDIENCE), goal.cell)
        assertEquals(listOf(34, 35, 36, 37), goal.tagIds)
        assertEquals(90.0, goal.goalRobot.x, 1e-6)
        assertEquals(12.0, goal.goalRobot.y, 1e-6)
        assertEquals(HiveTestFrames.RAISED_GOAL_HEIGHT_IN, goal.heightIn, 1e-6)
        assertEquals(Math.hypot(90.0, 12.0), goal.horizontalDistanceIn, 1e-6)
        assertEquals(Math.atan2(12.0, 90.0), goal.turretBearingRad, 1e-9)
        assertEquals(0.0, goal.spreadIn, 1e-6)
        assertFalse(goal.reprojected)

        assertTrue(rig.tracker.rows.all { it.heightClass == TagHeightClass.RAISED && it.usedInFusion })
        assertEquals(HiveTestFrames.RAISED_GOAL_HEIGHT_IN - 7.1874, rig.tracker.rows.first().tagRobot.z, 1e-6)
    }

    @Test
    fun framesArePlacedAtTheirEstimatedCaptureTime() {
        val rig = HiveRig()
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE), ageMs = 10L, captureMs = 20.0, targetingMs = 5.0)

        assertEquals(rig.clock.now - 35_000_000L, rig.requestedAngleTimes.single())
        assertEquals(35.0, rig.tracker.goal(Alliance.RED)!!.ageMs, 1e-6)
        assertTrue(rig.tracker.observedWithinMs(Alliance.RED, 40.0))
        assertFalse(rig.tracker.observedWithinMs(Alliance.BLUE, 40.0))
    }

    @Test
    fun anOutlierTagIsDroppedFromTheFusedGoal() {
        val rig = HiveRig()
        rig.frame(
            cluster(cell(Alliance.RED, CellLocation.AUDIENCE), Vec3(80.0, 0.0, 60.0), perturbIn = mapOf(36 to Vec3(10.0, 0.0, 0.0))),
        )

        val goal = rig.tracker.goal(Alliance.RED)!!
        assertEquals(listOf(34, 35, 37), goal.tagIds)
        assertEquals(0.0, goal.goalRobot.y, 1e-6)
        val outlier = rig.tracker.rows.single { it.tag.id == 36 }
        assertFalse(outlier.usedInFusion)
        assertEquals(10.0, outlier.deviationIn, 1e-6)
    }

    @Test
    fun loweredCellTagsAloneRevealTheRaisedCell() {
        val rig = HiveRig(priors = emptyMap())
        repeat(2) { rig.frame(lowered(Alliance.BLUE, CellLocation.AUDIENCE)) }

        assertEquals(HiveState.FAR_RAISED, rig.tracker.state(Alliance.BLUE))
        assertEquals(0, rig.tracker.tipCount(Alliance.BLUE))
        assertNull("the goal CELL itself was not seen", rig.tracker.goal(Alliance.BLUE))
    }

    @Test
    fun theGoalIsTheRaisedCellNotTheLoweredOne() {
        val rig = HiveRig()
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE, 70.0, 10.0) + lowered(Alliance.RED, CellLocation.FAR, 70.0, -10.0))

        val goal = rig.tracker.goal(Alliance.RED)!!
        assertEquals(CellLocation.AUDIENCE, goal.cell.location)
        assertEquals(10.0, goal.goalRobot.y, 1e-6)
        assertEquals(2, rig.tracker.cellGoals.size)
    }

    @Test
    fun aConfirmedTipMovesTheGoalAndIsReported() {
        val rig = HiveRig()
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE))
        val tipped = lowered(Alliance.RED, CellLocation.AUDIENCE, 70.0, 10.0) + raised(Alliance.RED, CellLocation.FAR, 70.0, -10.0)

        rig.frame(tipped)
        assertEquals("not yet confirmed; the audience goal is held", CellLocation.AUDIENCE, rig.tracker.goal(Alliance.RED)!!.cell.location)
        assertTrue(rig.events.isEmpty())

        rig.frame(tipped)
        val goal = rig.tracker.goal(Alliance.RED)!!
        assertEquals(CellLocation.FAR, goal.cell.location)
        assertEquals(-10.0, goal.goalRobot.y, 1e-6)
        assertEquals(1, rig.tracker.tipCount(Alliance.RED))
        assertNotNull(rig.tracker.lastTipNanos(Alliance.RED))
        assertEquals(listOf("HIVE TIP: RED audience→far"), rig.events)
    }

    @Test
    fun tagsNearTheThresholdCastNoVote() {
        val rig = HiveRig()
        val nearThreshold = cluster(cell(Alliance.RED, CellLocation.FAR), Vec3(80.0, 0.0, 43.95 + 7.1874))
        repeat(5) { rig.frame(nearThreshold) }

        assertTrue(rig.tracker.rows.all { it.heightClass == TagHeightClass.AMBIGUOUS })
        assertEquals(HiveState.AUDIENCE_RAISED, rig.tracker.state(Alliance.RED))
        assertTrue(rig.tracker.stateAssumed(Alliance.RED))
    }

    @Test
    fun goalsExpireAfterTheLostTimeout() {
        val rig = HiveRig()
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE))
        rig.idle(HiveConfig.lostTimeoutMs.toDouble())
        assertNotNull(rig.tracker.goal(Alliance.RED))
        rig.idle(0.001)
        assertNull(rig.tracker.goal(Alliance.RED))
    }

    @Test
    fun aHeldGoalIsReprojectedOntoTheCurrentPose() {
        val rig = HiveRig()
        rig.pose = Pose(10.0, 20.0, 0.0)
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE, 50.0, 0.0))
        rig.pose = Pose(10.0, 20.0, PI / 2)
        rig.idle(50.0)

        val goal = rig.tracker.goal(Alliance.RED)!!
        assertTrue(goal.reprojected)
        assertEquals(0.0, goal.goalRobot.x, 1e-6)
        assertEquals(-50.0, goal.goalRobot.y, 1e-6)
        assertEquals(-PI / 2, goal.robotBearingRad, 1e-9)
        assertEquals(HiveTestFrames.RAISED_GOAL_HEIGHT_IN, goal.heightIn, 1e-6)
    }

    @Test
    fun bothAlliancesAreTrackedFromOneFrame() {
        val rig = HiveRig()
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE, 80.0, 15.0) + raised(Alliance.BLUE, CellLocation.FAR, 80.0, -15.0))

        assertEquals(15.0, rig.tracker.goal(Alliance.RED)!!.goalRobot.y, 1e-6)
        assertEquals(-15.0, rig.tracker.goal(Alliance.BLUE)!!.goalRobot.y, 1e-6)
        assertFalse(rig.tracker.stateAssumed(Alliance.RED))
        assertFalse(rig.tracker.stateAssumed(Alliance.BLUE))
    }

    @Test
    fun nonSeasonTagsAndUnsolvedPosesAreIgnored() {
        val rig = HiveRig()
        rig.frame(
            listOf(
                fiducial(12, Vec3(0.0, -50.0, 80.0)),
                fiducial(34, Vec3(0.0, -50.0, 80.0), family = "16H5C"),
                fiducial(35, Vec3(0.0, -50.0, 80.0)).copy(targetPoseCameraSpace = null),
            ),
        )
        assertEquals(1L, rig.tracker.framesProcessed)
        assertTrue(rig.tracker.rows.isEmpty())
        assertNull(rig.tracker.goal(Alliance.RED))
    }

    @Test
    fun repeatedPollsWrongPipelinesAndStaleFramesAreNotProcessed() {
        val rig = HiveRig()
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE))
        rig.idle(5.0)
        assertEquals(1L, rig.tracker.framesProcessed)
        assertFalse(rig.tracker.newFrameThisTick)

        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE), pipeline = 3)
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE), ageMs = 150L)
        assertEquals(1L, rig.tracker.framesProcessed)
    }

    @Test
    fun framesWithoutATurretAngleAreDroppedAndCounted() {
        val rig = HiveRig()
        rig.turretAngle = null
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE))
        assertEquals(1L, rig.tracker.framesWithoutTurretAngle)
        assertEquals(0L, rig.tracker.framesProcessed)
        assertNull(rig.tracker.goal(Alliance.RED))
    }

    @Test
    fun logsGoalsStatesAndPerTagRows() {
        val rig = HiveRig()
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE, 72.0, 0.0))
        val log = RecordingStateLog()
        rig.tracker.logState(log)

        assertEquals("AUDIENCE_RAISED", log.channels["RED/state"])
        assertEquals(true, log.channels["RED/visible"])
        assertEquals(72.0, log.channels["RED/distanceIn"] as Double, 1e-6)
        assertEquals("34,35,36,37", log.channels["RED/tagIds"])
        assertEquals("FAR_RAISED", log.channels["BLUE/state"])
        assertEquals(true, log.channels["BLUE/stateAssumed"])
        assertTrue((log.channels["BLUE/distanceIn"] as Double).isNaN())
        assertArrayEquals(doubleArrayOf(34.0, 35.0, 36.0, 37.0), log.channels["tags/id"] as DoubleArray, 0.0)
        assertArrayEquals(doubleArrayOf(1.0, 1.0, 1.0, 1.0), log.channels["tags/class"] as DoubleArray, 0.0)
        assertEquals(false, log.channels["mount/measured"])
    }

    @Test
    fun withoutTagsTheAimGoalIsTheFieldGoalPlacedWithThePose() {
        val rig = HiveRig()
        rig.pose = Pose(58.0, 20.0, PI / 2)
        rig.idle(20.0)

        assertNull(rig.tracker.goal(Alliance.RED))
        val aim = rig.tracker.aimGoal(Alliance.RED)!!
        assertEquals(GoalSource.ODOMETRY, aim.source)
        assertEquals(CellLocation.AUDIENCE, aim.cell.location)
        assertEquals(HiveField.CENTRE_IN - 15.8 - 20.0, aim.goalRobot.x, 1e-9)
        assertEquals(0.0, aim.robotBearingRad, 1e-9)
        assertEquals(HiveField.RAISED_OPENING_HEIGHT_IN, aim.heightIn, 0.0)
        assertTrue(aim.tagIds.isEmpty())
    }

    @Test
    fun aKnownHiveStateBeatsTheRobotsSide() {
        val rig = HiveRig()
        rig.pose = Pose(58.0, 120.0, 0.0)
        rig.idle(20.0)
        assertEquals("match setup: red audience raised", CellLocation.AUDIENCE, rig.tracker.aimGoal(Alliance.RED)!!.cell.location)
    }

    @Test
    fun anUnknownHiveStateAimsAtTheCellOnTheRobotsSide() {
        val rig = HiveRig(priors = emptyMap())
        rig.pose = Pose(58.0, 120.0, 0.0)
        rig.idle(20.0)
        assertEquals(CellLocation.FAR, rig.tracker.aimGoal(Alliance.RED)!!.cell.location)
        rig.pose = Pose(58.0, 20.0, 0.0)
        rig.idle(20.0)
        assertEquals(CellLocation.AUDIENCE, rig.tracker.aimGoal(Alliance.RED)!!.cell.location)
    }

    @Test
    fun visionTakesOverAndOdometryReturnsAfterTheLostTimeout() {
        val rig = HiveRig()
        rig.pose = Pose(58.0, 20.0, PI / 2)
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE, 30.0, 3.0))

        val seen = rig.tracker.aimGoal(Alliance.RED)!!
        assertEquals(GoalSource.VISION, seen.source)
        assertEquals(30.0, seen.goalRobot.x, 1e-6)
        assertEquals(3.0, seen.goalRobot.y, 1e-6)

        rig.idle(HiveConfig.lostTimeoutMs + 1.0)
        assertEquals(GoalSource.ODOMETRY, rig.tracker.aimGoal(Alliance.RED)!!.source)
    }

    @Test
    fun withoutAPoseThereIsNoOdometryAim() {
        val rig = HiveRig()
        rig.idle(20.0)
        assertNull(rig.tracker.aimGoal(Alliance.RED))

        rig.pose = Pose(Double.NaN, 20.0, 0.0)
        rig.idle(20.0)
        assertNull(rig.tracker.aimGoal(Alliance.RED))
    }

    @Test
    fun logsTheAimSourceAndTheFieldGoalError() {
        val rig = HiveRig()
        rig.pose = Pose(58.0, 20.0, PI / 2)
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE, HiveField.CENTRE_IN - 15.8 - 20.0, 0.0))
        val log = RecordingStateLog()
        rig.tracker.logState(log)

        assertEquals("VISION", log.channels["RED/aimSource"])
        assertEquals(0.0, log.channels["RED/aimRobotBearingDeg"] as Double, 1e-6)
        assertEquals(
            HiveTestFrames.RAISED_GOAL_HEIGHT_IN - HiveField.RAISED_OPENING_HEIGHT_IN,
            log.channels["RED/fieldGoalErrorIn"] as Double,
            1e-6,
        )
        assertEquals("ODOMETRY", log.channels["BLUE/aimSource"])
        assertTrue((log.channels["BLUE/fieldGoalErrorIn"] as Double).isNaN())
    }

    @Test
    fun healthNamesAnUnmeasuredMount() {
        val rig = HiveRig()
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE, 72.0, 0.0))
        assertEquals(
            "camera mount not measured; RED audience up, goal 72 in; BLUE far up (assumed), no goal",
            rig.tracker.health(),
        )
        HiveConfig.mountMeasured = true
        assertTrue(rig.tracker.health().startsWith("RED audience up"))
    }

    private class RecordingStateLog : StateLog {
        val channels = mutableMapOf<String, Any>()
        override fun put(channel: String, value: Double) { channels[channel] = value }
        override fun put(channel: String, value: Long) { channels[channel] = value }
        override fun put(channel: String, value: Boolean) { channels[channel] = value }
        override fun put(channel: String, value: String) { channels[channel] = value }
        override fun put(channel: String, value: DoubleArray) { channels[channel] = value }
    }
}
