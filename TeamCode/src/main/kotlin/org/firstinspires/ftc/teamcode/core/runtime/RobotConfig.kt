package org.firstinspires.ftc.teamcode.core.runtime

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

    /**
     * Identifies which robot the on-hub tuning file belongs to. ConfigStore
     * ignores `/sdcard/FIRST/config/tuning.properties` when its recorded
     * schema differs from this string, so values tuned for a different robot
     * cannot win over this robot's compiled defaults.
     *
     * Bump it whenever the tuned values stop applying:
     *  - a new season fork (last season's tuning is meaningless), and
     *  - **the sensorbot → competition-robot swap**, since the Control Hub
     *    usually moves between chassis and carries its tuning file along.
     *    A light sensorbot's DriveConfig on a heavy competition robot is
     *    silently wrong, not an error.
     */
    const val CONFIG_SCHEMA = "biobuzz-sensorbot-v1"

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

    /** Game-level constants that change every season. Edit when the new game launches. */
    object Field {
        /** Distance from one end of the field to the other along the x-axis, in inches. */
        const val LENGTH_INCHES = 141.5

        /**
         * How RED coordinates map onto BLUE this season — reflection or 180°
         * rotation. Check the game manual's field drawings when the game
         * launches; getting this wrong silently breaks every BLUE auton path.
         */
        val SYMMETRY = org.firstinspires.ftc.teamcode.core.util.FieldSymmetry.MIRROR

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
         * Pedro's frame. Verify with the axis check in OPERATIONS.md.
         */
        const val FIELD_VIEW_QUARTER_TURNS = 1
    }
}
