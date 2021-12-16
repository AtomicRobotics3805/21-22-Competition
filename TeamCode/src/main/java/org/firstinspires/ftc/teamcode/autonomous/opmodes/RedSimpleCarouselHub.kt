package org.firstinspires.ftc.teamcode.autonomous.opmodes

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
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Autonomous(group = "Red", name = "Red Competition Autonomous")
@Disabled
class RedSimpleCarouselHub: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = Constants.Color.RED
        TrajectoryFactory.initializeStartPositions()
        Constants.startPose = TrajectoryFactory.simpleCarouselStartPose

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

        while (!isStarted) {
            //CommandScheduler.run()
        }

        CommandScheduler.commandsToSchedule += sequential {
            //+ObjectDetectionMB1220.DetectCommand()
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