package org.firstinspires.ftc.teamcode.core.pathing

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.Vector2d

/**
 * The geometry boundary between the framework and Pedro. Conversions live
 * here (plus the drive subsystem / Pedro `Localizer` implementations that
 * adapt whole APIs); everything else imports only
 * [org.firstinspires.ftc.teamcode.core.geometry] types. Both sides use the
 * same units and conventions, so these are field-for-field copies.
 */

fun Pose2d.toPedro(): Pose = Pose(x, y, heading)

fun Pose.toCore(): Pose2d = Pose2d(x, y, heading)

fun Vector.toCore(): Vector2d = Vector2d(xComponent, yComponent)
