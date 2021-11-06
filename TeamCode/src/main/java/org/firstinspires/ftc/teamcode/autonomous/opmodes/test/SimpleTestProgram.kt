package org.firstinspires.ftc.teamcode.autonomous.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name="Simple Test Program")
class SimpleTestProgram: LinearOpMode() {
    override fun runOpMode() {
        telemetry.addData("test", TestObject.test)
        telemetry.update()
        /*
        try {
            Constants.test = 5
        }
        catch(e: Exception) {
            telemetry.addData("Error", e.message)
            telemetry.update()
        }*/
        waitForStart()
    }
}