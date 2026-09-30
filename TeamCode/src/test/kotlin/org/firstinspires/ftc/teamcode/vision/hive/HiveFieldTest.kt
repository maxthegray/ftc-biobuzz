package org.firstinspires.ftc.teamcode.vision.hive

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.Alliance as FieldAlliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Cell
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.CellLocation
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class HiveFieldTest {

    @Test
    fun blueGoalsAreRedGoalsUnderTheSeasonSymmetry() {
        for (location in CellLocation.entries) {
            val red = HiveField.goal(Cell(Alliance.RED, location))
            val opposite = if (location == CellLocation.AUDIENCE) CellLocation.FAR else CellLocation.AUDIENCE
            val blue = HiveField.goal(Cell(Alliance.BLUE, opposite))
            val mirrored = FieldAlliance.BLUE.mirror(Pose(red.x, red.y, 0.0))
            assertEquals(blue.x, mirrored.x(), 1e-9)
            assertEquals(blue.y, mirrored.y(), 1e-9)
            assertEquals(red.z, blue.z, 0.0)
        }
    }

    @Test
    fun theRedHiveIsOnTheAudiencesLeftAndCellsSitOnTheirOwnSide() {
        val redAudience = HiveField.goal(Cell(Alliance.RED, CellLocation.AUDIENCE))
        val redFar = HiveField.goal(Cell(Alliance.RED, CellLocation.FAR))
        assertEquals(58.0, redAudience.x, 1e-9)
        assertEquals(83.5, HiveField.goal(Cell(Alliance.BLUE, CellLocation.FAR)).x, 1e-9)
        assertEquals(HiveField.CENTRE_IN - 15.8, redAudience.y, 1e-9)
        assertEquals(HiveField.CENTRE_IN + 15.8, redFar.y, 1e-9)
        assertTrue(redAudience.z > 53.5 && redAudience.z < 65.6)
    }

    @Test
    fun theCellOnSideFollowsTheRobotsHalf() {
        assertEquals(CellLocation.AUDIENCE, HiveField.cellOnSide(20.0))
        assertEquals(CellLocation.FAR, HiveField.cellOnSide(120.0))
        assertEquals(CellLocation.FAR, HiveField.cellOnSide(HiveField.CENTRE_IN))
    }

    @Test
    fun robotAndFieldFramesRoundTrip() {
        val pose = Pose(30.0, 40.0, 1.1)
        val field = Vec3(70.0, 90.0, 59.0)
        val robot = HiveField.toRobot(field, pose)
        val back = HiveField.toField(robot, pose)
        assertEquals(field.x, back.x, 1e-9)
        assertEquals(field.y, back.y, 1e-9)
        assertEquals(field.z, robot.z, 0.0)

        val ahead = HiveField.toRobot(Vec3(30.0, 60.0, 0.0), Pose(30.0, 40.0, Math.PI / 2))
        assertEquals(20.0, ahead.x, 1e-9)
        assertEquals(0.0, ahead.y, 1e-9)
    }
}
