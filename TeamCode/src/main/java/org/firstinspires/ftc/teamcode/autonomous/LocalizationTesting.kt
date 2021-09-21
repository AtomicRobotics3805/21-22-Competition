package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive

@Autonomous
class LocalizationTesting: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        MecanumDrive.initialize()

        waitForStart()
        while(opModeIsActive()) {
            MecanumDrive.periodic()
        }
    }
}