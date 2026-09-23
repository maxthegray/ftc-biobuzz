package org.firstinspires.ftc.teamcode.subsystems.turret

/**
 * The turret's measured angle at a past moment, radians, counterclockwise
 * from the robot's front. The camera rides on the turret, so every tag frame
 * is placed on the robot with the angle at its capture time, not the command.
 * Null when that moment is outside the retained history.
 */
fun interface TurretAngleSource {
    fun angleAtRad(timestampNanos: Long): Double?
}

/** A turret held at one angle: a bench survey on a tripod, or tests. */
class FixedTurretAngle(private val angleRad: Double = 0.0) : TurretAngleSource {
    override fun angleAtRad(timestampNanos: Long): Double = angleRad
}
