package org.firstinspires.ftc.teamcode.vision.hive

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightFiducial
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightReading
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSource
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretAngleSource
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.CellLocation
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

class HiveGoalSubsystemTest {

    private val clock = FakeClock()
    private val source = FakeLimelightSource()
    private val limelight = LimelightSubsystem(source = source, clock = clock)
    private val requestedAngles = ArrayList<Long>()
    private var turretAngle: Double? = 0.0
    private val turret = TurretAngleSource { t -> requestedAngles += t; turretAngle }
    private val events = ArrayList<String>()
    private val hive = HiveGoalSubsystem(limelight, turret, HiveGoalSubsystem.MATCH_SETUP, eventSink = events::add, clock = clock)
    private var frameTs = 100.0

    @Before
    fun setUp() {
        HiveGoalConfig.resetDefaults()
        TurretCameraMountConfig.resetDefaults()
        limelight.init(HardwareMap(null, null))
        source.isConnected = true
    }

    @After
    fun tearDown() {
        HiveGoalConfig.resetDefaults()
        TurretCameraMountConfig.resetDefaults()
    }

    @Test
    fun aFreshFrameBecomesAGoalAtItsCaptureTime() {
        tick(raised(Alliance.RED, CellLocation.AUDIENCE, 72.0, 0.0), ageMs = 10L, captureMs = 20.0, targetingMs = 5.0)

        val goal = hive.goal(Alliance.RED)
        assertNotNull(goal)
        assertEquals(72.0, goal!!.horizontalDistanceIn, 1e-6)
        assertEquals(clock.now - 35_000_000L, requestedAngles.single())
        assertEquals(35.0, goal.ageMs, 1e-6)
        assertEquals(HiveState.AUDIENCE_RAISED, hive.state(Alliance.RED))
        assertFalse(hive.stateAssumed(Alliance.RED))
        assertTrue(hive.stateAssumed(Alliance.BLUE))
        assertNull(hive.goal(Alliance.BLUE))
        assertTrue(hive.observedWithinMs(Alliance.RED, 40.0))
    }

    @Test
    fun aRepeatedPollIsNotANewFrame() {
        tick(raised(Alliance.RED, CellLocation.AUDIENCE))
        limelight.periodic()
        hive.periodic()
        assertEquals(1L, hive.tracker.framesProcessed)
        assertFalse(hive.newFrameThisTick)
    }

    @Test
    fun wrongPipelineAndStaleFramesAreIgnored() {
        tick(raised(Alliance.RED, CellLocation.AUDIENCE), pipeline = 3)
        assertEquals(0L, hive.tracker.framesProcessed)

        tick(raised(Alliance.RED, CellLocation.AUDIENCE), ageMs = 150L)
        assertEquals(0L, hive.tracker.framesProcessed)
    }

    @Test
    fun framesWithoutATurretAngleAreDroppedAndCounted() {
        turretAngle = null
        tick(raised(Alliance.RED, CellLocation.AUDIENCE))
        assertEquals(1L, hive.framesWithoutTurretAngle)
        assertEquals(0L, hive.tracker.framesProcessed)
        assertNull(hive.goal(Alliance.RED))
    }

    @Test
    fun theGoalIsLostAfterTheTimeoutWithoutNewFrames() {
        tick(raised(Alliance.RED, CellLocation.AUDIENCE), ageMs = 0L)
        assertNotNull(hive.goal(Alliance.RED))
        clock.advanceMs(HiveGoalConfig.lostTimeoutMs + 1.0)
        limelight.periodic()
        hive.periodic()
        assertNull(hive.goal(Alliance.RED))
    }

    @Test
    fun tipsAreCountedAndReportedAsEvents() {
        val tipped = lowered(Alliance.RED, CellLocation.AUDIENCE) + raised(Alliance.RED, CellLocation.FAR)
        repeat(HiveGoalConfig.tipConfirmFrames) { tick(tipped) }

        assertEquals(HiveState.FAR_RAISED, hive.state(Alliance.RED))
        assertEquals(1, hive.tipCount(Alliance.RED))
        assertNotNull(hive.lastTipNanos(Alliance.RED))
        assertEquals(1, events.size)
        assertTrue(events.single(), events.single().startsWith("HIVE TIP: RED audience→far"))
    }

    @Test
    fun logsGoalsStatesAndPerTagRows() {
        tick(raised(Alliance.RED, CellLocation.AUDIENCE, 72.0, 0.0))
        val log = RecordingStateLog()
        hive.logState(log)

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
    fun healthNamesAnUnmeasuredMount() {
        tick(raised(Alliance.RED, CellLocation.AUDIENCE, 72.0, 0.0))
        assertEquals(
            "camera mount not measured; RED audience up, goal 72 in; BLUE far up (assumed), no goal",
            hive.health(),
        )
        TurretCameraMountConfig.measured = true
        hive.periodic()
        assertTrue(hive.health().startsWith("RED audience up"))
    }

    private fun tick(
        fiducials: List<LimelightFiducial>,
        ageMs: Long = 10L,
        captureMs: Double = 0.0,
        targetingMs: Double = 0.0,
        pipeline: Int = 0,
    ) {
        clock.advanceMs(20.0)
        frameTs += 20.0
        source.reading = LimelightReading(
            receiptTimestampMs = frameTs.toLong(),
            ageMs = ageMs,
            valid = fiducials.isNotEmpty(),
            pipelineIndex = pipeline,
            pipelineType = "pipe_fiducial",
            captureLatencyMs = captureMs,
            targetingLatencyMs = targetingMs,
            limelightTimestampMs = frameTs,
            fiducials = fiducials,
        )
        limelight.periodic()
        hive.periodic()
    }

    private class RecordingStateLog : StateLog {
        val channels = mutableMapOf<String, Any>()
        override fun put(channel: String, value: Double) { channels[channel] = value }
        override fun put(channel: String, value: Long) { channels[channel] = value }
        override fun put(channel: String, value: Boolean) { channels[channel] = value }
        override fun put(channel: String, value: String) { channels[channel] = value }
        override fun put(channel: String, value: DoubleArray) { channels[channel] = value }
    }

    private class FakeLimelightSource : LimelightSource {
        override var isRunning = false
        override var isConnected = false
        var reading = LimelightReading()

        override fun setPollRateHz(rateHz: Int) {}
        override fun pipelineSwitch(index: Int): Boolean = true
        override fun start() {
            isRunning = true
        }
        override fun latestReading(): LimelightReading = reading
        override fun stop() {
            isRunning = false
        }
    }
}
