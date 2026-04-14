package org.firstinspires.ftc.teamcode.starter.config

/**
 * Central list of hardware-map names + wiring-level knobs.
 *
 * This file holds the names the robot's active "Configuration" on the
 * Driver Station must use. Everything in [org.firstinspires.ftc.teamcode
 * .pedroPathing.Constants] is physical (mass, zero-power accel, pod
 * offsets); everything here is identity (what's this device called in the
 * config xml).
 *
 * Changing a name here must be matched on the Robot Controller's
 * "Configure Robot" screen — there is no runtime magic.
 */
object RobotConfig {

    object Drive {
        const val FRONT_LEFT_MOTOR = "frontLeftMotor"
        const val FRONT_RIGHT_MOTOR = "frontRightMotor"
        const val BACK_LEFT_MOTOR = "backLeftMotor"
        const val BACK_RIGHT_MOTOR = "backRightMotor"
    }

    object Localization {
        /**
         * Hardware-map name of the GoBilda Pinpoint. Named "sensor_otos"
         * because the robot kept the config entry after migrating off the
         * SparkFun OTOS — change this if you re-name the device on the
         * Driver Station.
         */
        const val PINPOINT = "sensor_otos"
    }

    object Vision {
        const val WEBCAM = "hsc"
    }

    /** Game-level constants that change every season. Edit when the new game launches. */
    object Field {
        /** Distance from one end of the field to the other along the x-axis, in inches. */
        const val LENGTH_INCHES = 141.5
    }
}
