package org.firstinspires.ftc.teamcode.vision

import com.pedropathing.math.Pose
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.core.Alliance
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class GoalAimTest {

    private val eps = 1e-9

    @Test
    fun noOwnTagsIsNoGoal() {
        assertNull(GoalAim.goal(Alliance.RED, emptyList()))
        assertNull(GoalAim.goal(Alliance.RED, listOf(TagSighting(40, 1.0, 10.0), TagSighting(7, 0.0, 0.0))))
    }

    @Test
    fun goalIsTheAverageOfOneCellsTags() {
        val goal = GoalAim.goal(Alliance.RED, listOf(TagSighting(31, -2.0, 10.0), TagSighting(32, 4.0, 12.0), TagSighting(40, 30.0, 20.0)))!!
        assertEquals(listOf(31, 32), goal.tagIds)
        assertEquals(1.0, goal.txDeg, eps)
        assertEquals(11.0, goal.tyDeg, eps)
    }

    @Test
    fun higherCellWinsWhenBothShow() {
        val tags = listOf(TagSighting(38, -10.0, -5.0), TagSighting(43, 8.0, 15.0), TagSighting(44, 10.0, 15.0))
        assertEquals(listOf(43, 44), GoalAim.goal(Alliance.BLUE, tags)!!.tagIds)
    }

    @Test
    fun hivePivotsSitEitherSideOfTheFieldCentre() {
        assertEquals(58.0, GoalAim.pivot(Alliance.RED).x(), eps)
        assertEquals(83.5, GoalAim.pivot(Alliance.BLUE).x(), eps)
        assertEquals(70.75, GoalAim.pivot(Alliance.BLUE).y(), eps)
    }

    @Test
    fun pivotBearingIsRelativeToTheRobotHeading() {
        assertEquals(0.0, GoalAim.pivotBearingRad(Pose(58.0, 20.0, PI / 2), Alliance.RED), eps)
        assertEquals(PI / 2, GoalAim.pivotBearingRad(Pose(58.0, 20.0, 0.0), Alliance.RED), eps)
        assertEquals(-PI / 2, GoalAim.pivotBearingRad(Pose(83.5, 120.0, 0.0), Alliance.BLUE), eps)
    }
}
