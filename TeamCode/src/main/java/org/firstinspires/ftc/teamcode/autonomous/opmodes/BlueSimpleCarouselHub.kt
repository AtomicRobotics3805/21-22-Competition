package org.firstinspires.ftc.teamcode.autonomous.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.AutoRoutines
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetectionMB1220
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.*
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.toRadians
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Autonomous(group = "Blue", name = "Blue Competition Autonomous")
class BlueSimpleCarouselHub: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = Constants.Color.BLUE
        TrajectoryFactory.initializeStartPositions()
        Constants.startPose = TrajectoryFactory.simpleCarouselStartPose

        MecanumDrive.initialize()
        Arm.initialize()
        Bucket.initialize()
        Carousel.initialize()
        Intake.initialize()
        DeadWheelServo.initialize()
        BucketLatch.initialize()
        ObjectDetectionMB1220.initialize()
        Cap.initialize()
        TrajectoryFactory.initializeTrajectories()

        CommandScheduler.registerSubsystems(MecanumDrive, Arm, Bucket, Carousel, Intake)
        CommandScheduler.cancelAll()

        while (!isStarted) {
            //CommandScheduler.run()
        }

        CommandScheduler.commandsToSchedule += sequential {
            +ObjectDetectionMB1220.DetectCommand()
            +AutoRoutines.simpleCarouselHubRoutine
        }
        /*CommandScheduler.commandsToSchedule += sequential {
            +MecanumDrive.turnRelative((-90.0).toRadians)
            +MecanumDrive.turnRelative(90.0.toRadians)
            +MecanumDrive.turnRelative(90.0.toRadians)
            +MecanumDrive.turnRelative(90.0.toRadians)
        }*/

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}