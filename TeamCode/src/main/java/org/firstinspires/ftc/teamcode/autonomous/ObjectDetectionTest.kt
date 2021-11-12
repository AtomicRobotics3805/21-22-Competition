package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants.opMode

@Autonomous(name="Object Detection Test")
class ObjectDetectionTest : LinearOpMode() {
    override fun runOpMode() {
        opMode = this

        telemetry.addData("Status", "Ready")
        telemetry.update()

        waitForStart()

        ObjectDetection.detect.execute()
    }
}