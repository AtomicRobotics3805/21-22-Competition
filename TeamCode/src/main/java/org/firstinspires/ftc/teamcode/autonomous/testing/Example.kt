package org.firstinspires.ftc.teamcode.autonomous.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Autonomous
@Disabled
class Example : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        while (opModeIsActive()) {
            var leftStickY = gamepad1.left_stick_y
            var rightStickY = gamepad1.right_stick_y

        }
    }
}