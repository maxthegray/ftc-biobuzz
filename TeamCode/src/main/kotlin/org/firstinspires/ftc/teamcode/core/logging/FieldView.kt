package org.firstinspires.ftc.teamcode.core.logging

import com.bylazar.field.FieldManager
import com.bylazar.field.PanelsField
import com.bylazar.field.Style
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.qualcomm.robotcore.util.RobotLog
import kotlin.math.cos
import kotlin.math.sin
import org.firstinspires.ftc.teamcode.core.runtime.DriveTelemetrySource
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * Live field view on the Panels dashboard for *real* op-modes (the upstream
 * Pedro `Tuning` op-mode draws its own): robot pose + heading every redraw,
 * plus the active path while the drive is following. This is the primary
 * "where does the robot think it is" debugging surface during auton
 * development and matches. Consumes [DriveTelemetrySource], so it never
 * touches Pedro types directly.
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

    fun draw(drive: DriveTelemetrySource?) {
        if (drive == null || failures >= MAX_FAILURES) return
        val now = clock.nanos()
        if (lastDrawNs != Long.MIN_VALUE && now - lastDrawNs < intervalNs) return
        lastDrawNs = now

        try {
            val manager = field ?: PanelsField.field.also {
                it.setOffsets(PanelsField.presets.PEDRO_PATHING)
                field = it
            }
            drawPath(manager, drive.currentPathPoints(PATH_SAMPLES))
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

    private fun drawPath(manager: FieldManager, points: List<Vector2D>) {
        // FieldManager.line() draws from the cursor but does not advance it,
        // so each segment re-anchors the cursor at the previous sample.
        manager.setStyle(pathStyle)
        var prev: Vector2D? = null
        for (point in points) {
            if (point.x().isNaN() || point.y().isNaN()) continue
            prev?.let {
                manager.moveCursor(it.x(), it.y())
                manager.line(point.x(), point.y())
            }
            prev = point
        }
    }

    private fun drawRobot(manager: FieldManager, pose: Pose) {
        val x = pose.x()
        val y = pose.y()
        val heading = pose.heading()
        if (x.isNaN() || y.isNaN() || heading.isNaN()) return

        manager.setStyle(robotStyle)
        manager.moveCursor(x, y)
        manager.circle(ROBOT_RADIUS)

        val hx = cos(heading) * ROBOT_RADIUS
        val hy = sin(heading) * ROBOT_RADIUS
        manager.setStyle(robotStyle)
        manager.moveCursor(x + hx / 2.0, y + hy / 2.0)
        manager.line(x + hx, y + hy)
    }

    private companion object {
        const val ROBOT_RADIUS = 9.0
        const val MAX_FAILURES = 5
        const val PATH_SAMPLES = 16
    }
}
