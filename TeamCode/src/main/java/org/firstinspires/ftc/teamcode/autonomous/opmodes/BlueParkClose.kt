package org.firstinspires.ftc.teamcode.autonomous.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.AutoRoutines
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Arm
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Carousel
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Autonomous(group = "Blue", name = "Blue Park Close")
class BlueParkClose: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = Constants.Color.BLUE
        Constants.startPose = TrajectoryFactory.closeParkStartPose

        MecanumDrive.initialize()
        Arm.initialize()
        Bucket.initialize()
        Carousel.initialize()
        Intake.initialize()
        TrajectoryFactory.initializeTrajectories()

        CommandScheduler.registerSubsystems(MecanumDrive, Arm, Bucket, Carousel, Intake)
        CommandScheduler.commands += AutoRoutines.parkCloseRoutine

        waitForStart()

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}