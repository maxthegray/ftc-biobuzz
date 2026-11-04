package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "Servo Test", group = "Diagnostics")
class ServoTestTeleOp : LinearOpMode() {
    override fun runOpMode() {
        val min = 0.0
        val max = 0.4
        val speed = 0.01
        // Encoder ticks per output rev: 28.0 for the goBILDA 6000 rpm motor, 103.8 for the 1620.
        val ticksPerRev = 28.0

        val servo = hardwareMap.get(Servo::class.java, "servo")
        val motor = hardwareMap.get(DcMotorEx::class.java, "shooter_motor")
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        var position = min
        var velocity = 0.0

        waitForStart()
        while (opModeIsActive()) {
            position = (position - gamepad1.left_stick_y * speed).coerceIn(min, max)
            servo.position = position

            if (gamepad1.dpadUpWasPressed()) velocity += 100
            if (gamepad1.dpadDownWasPressed()) velocity -= 100
            if (gamepad1.dpadRightWasPressed()) velocity += 10
            if (gamepad1.dpadLeftWasPressed()) velocity -= 10
            if (gamepad1.crossWasPressed()) velocity = 0.0
            motor.velocity = velocity

            telemetry.addData("position", position)
            val measured = motor.velocity
            telemetry.addData("target velocity (ticks/s)", velocity)
            telemetry.addData("measured velocity (ticks/s)", measured)
            telemetry.addData("target RPM (for 6000 rpm motor)", velocity / ticksPerRev * 60.0)
            telemetry.addData("measured RPM (for 6000 rpm motor)", measured / ticksPerRev * 60.0)
            telemetry.update()
        }
    }
}
