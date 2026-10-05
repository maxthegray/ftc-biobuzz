package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.Cell
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.CellLocation
import org.firstinspires.ftc.teamcode.vision.HiveConfig
import org.firstinspires.ftc.teamcode.vision.HiveRig
import org.firstinspires.ftc.teamcode.vision.HiveTestFrames.lowered
import org.firstinspires.ftc.teamcode.vision.HiveTestFrames.raised
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class HiveSurveyMemoryTest {
    private val savedConfig = ConfigSnapshot(HiveConfig)

    @After
    fun tearDown() = savedConfig.restore()

    @Test
    fun keepsEveryTagAndCellSeenAcrossFrames() {
        val rig = HiveRig(priors = emptyMap())
        val survey = HiveSurveyMemory()
        rig.frame(raised(Alliance.RED, CellLocation.AUDIENCE, 72.0, 0.0))
        survey.record(rig.tracker, rig.tracker.lastFrameCaptureNanos!!)
        rig.idle(1_980.0)
        rig.frame(lowered(Alliance.BLUE, CellLocation.FAR, 60.0, -20.0))
        survey.record(rig.tracker, rig.tracker.lastFrameCaptureNanos!!)

        assertNotNull(survey.tag(34))
        assertNotNull(survey.tag(45))
        assertNull(survey.tag(30))
        assertNotNull(survey.cell(Cell(Alliance.RED, CellLocation.AUDIENCE)))

        val m = survey.measurements(rig.tracker, rig.clock.now + 1_000_000_000L).toMap()
        assertEquals("UNKNOWN; tips 0", m["hive.RED"])
        val redAudience = m.getValue("cell.RED_AUDIENCE")
        assertTrue(redAudience, redAudience.startsWith("RAISED goal (72.0, 0.0, 60.0) in; horizontal 72.0 in"))
        assertTrue(redAudience, redAudience.endsWith("seen 3.0 s ago"))
        assertEquals("not seen", m["cell.RED_SCORING"])
        assertTrue(m.getValue("tag.45"), m.getValue("tag.45").startsWith("BLUE SCORING slot 4; LOWERED at 28.8 in"))
        assertEquals("RED SCORING slot 1; not seen", m["tag.30"])
        assertEquals(2 + 4 + 16, m.size)

        survey.reset()
        assertNull(survey.tag(34))
    }
}
