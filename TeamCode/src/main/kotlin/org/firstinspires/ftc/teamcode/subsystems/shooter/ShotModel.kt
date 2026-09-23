package org.firstinspires.ftc.teamcode.subsystems.shooter

/**
 * Flywheel speed for a shot at the HIVE goal. The input is
 * `GoalObservation.horizontalDistanceIn`: horizontal distance from the turret
 * axis to the raised CELL's opening centre, inches.
 *
 * Not yet calibrated: fit this from shots at measured distances. Until then it
 * returns null, and callers must not spin the flywheel from it.
 */
object ShotModel {
    fun rpmForDistance(distanceIn: Double): Double? = null
}
