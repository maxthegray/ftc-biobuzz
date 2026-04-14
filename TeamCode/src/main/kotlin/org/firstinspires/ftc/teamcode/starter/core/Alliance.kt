package org.firstinspires.ftc.teamcode.starter.core

import com.pedropathing.geometry.Pose

/**
 * Alliance / starting-side selection. Used by auton to mirror paths automatically
 * so you write one path file per routine and run it on either side.
 */
enum class Alliance {
    RED,
    BLUE;

    val sign: Double get() = if (this == RED) 1.0 else -1.0

    /**
     * Returns `pose` unchanged for RED; mirrors across the field for BLUE so a
     * single set of path poses can run on either alliance. Pedro's default mirror
     * axis is the standard FTC field (141.5" in Pedro coordinates).
     */
    fun mirror(pose: Pose): Pose =
        if (this == RED) pose else pose.mirror()
}
