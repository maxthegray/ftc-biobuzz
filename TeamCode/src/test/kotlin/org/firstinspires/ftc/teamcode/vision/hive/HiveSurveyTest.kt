package org.firstinspires.ftc.teamcode.vision.hive

import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.CellLocation
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.cell
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.lowered
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.mount
import org.firstinspires.ftc.teamcode.vision.hive.HiveTestFrames.raised
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class HiveSurveyTest {

    private val s = 1_000_000_000L

    @Test
    fun keepsEveryTagAndCellSeenAcrossFrames() {
        val tracker = HiveGoalTracker()
        val survey = HiveSurvey()
        tracker.update(TagFrame(raised(Alliance.RED, CellLocation.AUDIENCE, 72.0, 0.0), 0, 0.0), mount, HiveGoalSettings())
        survey.record(tracker, 0)
        tracker.update(TagFrame(lowered(Alliance.BLUE, CellLocation.FAR, 60.0, -20.0), 2 * s, 0.0), mount, HiveGoalSettings())
        survey.record(tracker, 2 * s)

        assertNotNull(survey.tag(34))
        assertNotNull(survey.tag(45))
        assertNull(survey.tag(30))
        assertNotNull(survey.cell(cell(Alliance.RED, CellLocation.AUDIENCE)))

        val m = survey.measurements(tracker, mount, 3 * s).toMap()
        assertEquals("UNKNOWN; tips 0", m["hive.RED"])
        assertTrue(m.getValue("cell.RED_AUDIENCE"), m.getValue("cell.RED_AUDIENCE").startsWith("RAISED goal (72.0, 0.0, 60.0) in; horizontal 72.0 in"))
        assertTrue(m.getValue("cell.RED_AUDIENCE").endsWith("seen 3.0 s ago"))
        assertEquals("not seen", m["cell.RED_SCORING"])
        assertTrue(m.getValue("tag.45"), m.getValue("tag.45").startsWith("BLUE SCORING slot 4; LOWERED at 28.8 in"))
        assertEquals("RED SCORING slot 1; not seen", m["tag.30"])
        assertEquals(2 + 4 + 16, m.size)

        survey.reset()
        assertNull(survey.tag(34))
    }
}
