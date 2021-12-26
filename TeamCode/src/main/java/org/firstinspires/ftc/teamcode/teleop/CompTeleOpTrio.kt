package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.trio.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms.*
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Suppress("unused")
@TeleOp(name = "Competition Testing")
@Disabled
class CompTeleOpTrio: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this

        MecanumDrive.initialize()
        Arm.initialize()
        Bucket.initialize()
        Carousel.initialize()
        Intake.initialize()
        OdometryServo.initialize()
        BucketLock.initialize()
        Controls.registerGamepads()
        CommandScheduler.registerSubsystems(MecanumDrive, Arm, Bucket, Carousel, Intake)
        CommandScheduler.cancelAll()

        waitForStart()

        Controls.registerTrioCommands()

        while (opModeIsActive()) {
            CommandScheduler.run()
            telemetry.addData("Position", MecanumDrive.poseEstimate)
            telemetry.update()
        }
    }
}