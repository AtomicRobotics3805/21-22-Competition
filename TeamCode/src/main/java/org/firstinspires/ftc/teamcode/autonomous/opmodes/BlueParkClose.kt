package org.firstinspires.ftc.teamcode.autonomous.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.AutoRoutines
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetectionMB1220
import org.firstinspires.ftc.teamcode.subsystems.trio.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.trio.*
import org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms.*
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Autonomous(group = "Blue", name = "Blue Park Close")
@Disabled
class BlueParkClose: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = Constants.Color.BLUE
        TrajectoryFactory.initializeStartPositions()
        Constants.startPose = Pose2d()

        MecanumDrive.initialize()
        Arm.initialize()
        Bucket.initialize()
        Carousel.initialize()
        Intake.initialize()
        OdometryServo.initialize()
        BucketLock.initialize()
        ObjectDetectionMB1220.initialize()
        CapArm.initialize()
        TrajectoryFactory.initializeTrajectories()

        CommandScheduler.registerSubsystems(MecanumDrive, Arm, Bucket, Carousel, Intake)
        CommandScheduler.cancelAll()
        CommandScheduler.commandsToSchedule += OdometryServo.up
        CommandScheduler.commandsToSchedule += ObjectDetectionMB1220.DetectCommand()

        while (!isStarted) {
            CommandScheduler.run()
        }

        CommandScheduler.commandsToSchedule += AutoRoutines.testRoutine

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}