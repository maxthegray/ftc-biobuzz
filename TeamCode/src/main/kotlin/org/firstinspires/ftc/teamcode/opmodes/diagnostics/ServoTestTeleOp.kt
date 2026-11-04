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
            telemetry.addData("target velocity", velocity)
            telemetry.addData("measured velocity", motor.velocity)
            telemetry.update()
        }
    }
}
