package org.firstinspires.ftc.teamcode

/**
 * Central list of hardware-map names + wiring-level knobs.
 *
 * This file holds the names the robot's active "Configuration" on the
 * Driver Station must use. Everything in [org.firstinspires.ftc.teamcode
 * .pedro.Constants] is physical (motor directions, pod offsets, Foresight
 * tuning); everything here is identity (what's this device called in the
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
        /** Hardware-map name of the GoBilda Pinpoint. */
        const val PINPOINT = "pinpoint"
    }

    /** Intake motors; add a name when a second motor is fitted. */
    object Intake {
        val MOTORS = listOf("intakeMotor")
    }

    /** Middle motors that lift balls to the shooter, and the servo tab that blocks the turret entry. */
    object Transfer {
        val MOTORS = listOf("transferMotor")
        const val BLOCKER_SERVO = "blockerServo"
    }

    /** Game-level constants that change every season. Edit when the new game launches. */
    object Field {
        /** Distance from one end of the field to the other along the x-axis, in inches. */
        const val LENGTH_INCHES = 141.5

        /**
         * How RED coordinates map onto BLUE this season — reflection or 180°
         * rotation. Check the game manual's field drawings when the game
         * launches; getting this wrong silently breaks every BLUE auton path.
         *
         * BIOBUZZ is ROTATE: the red LOADING ZONE is on tile A5 and the blue
         * on F2, the red GARDEN on A1 and the blue on F6 (manual Figures 9-2,
         * 9-5), and the CELLs raised at setup are RED audience and BLUE far
         * (Figure 10-2). Only a 180° rotation maps each onto the other.
         */
        val SYMMETRY = org.firstinspires.ftc.teamcode.core.FieldSymmetry.ROTATE

        /**
         * Counter-clockwise quarter turns from Pedro's axes onto the FTC field
         * frame, used only by the flight recorder's `Field/Robot` channel so
         * AdvantageScope's 2D field (*Center/Rotated*) draws the robot where it
         * really is. Pedro: (0, 0) at the corner on the audience's left, +X
         * along the audience wall, +Y away from the audience. FTC: origin at
         * the centre, +Y from the red wall to the blue wall. So this is +1 when
         * the red wall is on the audience's left (DECODE 2025-26, where +X
         * points at the audience) and −1 when it is on the right (the usual
         * layout). Display only: paths, start poses and `Alliance` stay in
         * Pedro's frame. Verify with the axis check in dutchdocs/OPERATIONS.md.
         */
        const val FIELD_VIEW_QUARTER_TURNS = 1
    }
}
