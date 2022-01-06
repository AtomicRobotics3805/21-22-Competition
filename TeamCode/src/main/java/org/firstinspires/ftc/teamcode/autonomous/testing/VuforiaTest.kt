package org.firstinspires.ftc.teamcode.autonomous.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.subsystems.localization.VuforiaLocalizer

class VuforiaTest : LinearOpMode() {
    override fun runOpMode() {
        opMode = this

        VuforiaLocalizer.initialize()

        waitForStart()

        while(opModeIsActive()) {
            VuforiaLocalizer.update()
            telemetry.update()
        }
    }
}