package org.firstinspires.ftc.teamcode.core.logging

import com.bylazar.field.FieldManager
import com.bylazar.field.PanelsField
import com.bylazar.field.Style
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * Live field view on the Panels dashboard for *real* op-modes (the upstream
 * Pedro `Tuning` op-mode draws its own): robot pose + heading every redraw,
 * plus the active path while the drive is FOLLOWING. This is the primary
 * "where does the robot think it is" debugging surface during auton
 * development and matches.
 *
 * Redraws are throttled to [redrawIntervalMs] — pushing the field websocket
 * at loop rate is pure overhead. Drawing must never stop the robot: any
 * failure is counted, and the view disables itself after a few, mirroring
 * the telemetry-flush policy in
 * [org.firstinspires.ftc.teamcode.core.runtime.OpModeBase].
 */
class FieldView(
    redrawIntervalMs: Double = 100.0,
    private val clock: Clock = Clock.SYSTEM,
) {
    private val intervalNs = (redrawIntervalMs * 1_000_000.0).toLong()
    private var lastDrawNs = Long.MIN_VALUE
    private var failures = 0

    private var field: FieldManager? = null

    private val robotStyle = Style("", "#3F51B5", 0.75)
    private val pathStyle = Style("", "#4CAF50", 0.75)

    fun draw(drive: MecanumDriveSubsystem?) {
        if (drive == null || failures >= MAX_FAILURES) return
        val now = clock.nanos()
        if (lastDrawNs != Long.MIN_VALUE && now - lastDrawNs < intervalNs) return
        lastDrawNs = now

        try {
            val manager = field ?: PanelsField.field.also {
                it.setOffsets(PanelsField.presets.PEDRO_PATHING)
                field = it
            }
            if (drive.mode == MecanumDriveSubsystem.Mode.FOLLOWING) {
                drawActivePath(manager, drive)
            }
            drawRobot(manager, drive.pose)
            manager.update()
        } catch (t: Throwable) {
            failures++
            if (failures == MAX_FAILURES) {
                try {
                    RobotLog.ee("FieldView", t, "Field view disabled after $failures failures")
                } catch (_: Throwable) {
                }
            }
        }
    }

    private fun drawActivePath(manager: FieldManager, drive: MecanumDriveSubsystem) {
        val chain = drive.follower.currentPathChain
        if (chain != null) {
            for (i in 0 until chain.size()) drawPath(manager, chain.getPath(i))
        } else {
            drive.follower.currentPath?.let { drawPath(manager, it) }
        }
    }

    private fun drawPath(manager: FieldManager, path: Path) {
        val points = path.panelsDrawingPoints
        for (i in points.indices) {
            for (j in points[i].indices) {
                if (points[i][j].isNaN()) points[i][j] = 0.0
            }
        }
        manager.setStyle(pathStyle)
        manager.moveCursor(points[0][0], points[0][1])
        manager.line(points[1][0], points[1][1])
    }

    private fun drawRobot(manager: FieldManager, pose: Pose) {
        if (pose.x.isNaN() || pose.y.isNaN() || pose.heading.isNaN()) return

        manager.setStyle(robotStyle)
        manager.moveCursor(pose.x, pose.y)
        manager.circle(ROBOT_RADIUS)

        val heading = pose.headingAsUnitVector
        heading.magnitude = heading.magnitude * ROBOT_RADIUS
        val x1 = pose.x + heading.xComponent / 2.0
        val y1 = pose.y + heading.yComponent / 2.0
        manager.setStyle(robotStyle)
        manager.moveCursor(x1, y1)
        manager.line(pose.x + heading.xComponent, pose.y + heading.yComponent)
    }

    private companion object {
        const val ROBOT_RADIUS = 9.0
        const val MAX_FAILURES = 5
    }
}
