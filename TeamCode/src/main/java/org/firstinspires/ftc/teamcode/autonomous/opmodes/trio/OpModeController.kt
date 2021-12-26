package org.firstinspires.ftc.teamcode.autonomous.opmodes.trio

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetectionMB1220
import org.firstinspires.ftc.teamcode.subsystems.trio.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms.*
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

object OpModeController {
    fun initialize(opMode: LinearOpMode) {
        Constants.opMode = opMode

        MecanumDrive.initialize()
        Arm.initialize()
        Bucket.initialize()
        Carousel.initialize()
        Intake.initialize()
        OdometryServo.initialize()
        BucketLock.initialize()
        ObjectDetectionMB1220.initialize()
        CapArm.initialize()
        OdometryServo.initialize()
        TrajectoryFactory.initializeTrioTrajectories()

        CommandScheduler.registerSubsystems(MecanumDrive, Arm, Bucket, Carousel, Intake)
        CommandScheduler.cancelAll()

        while (!opMode.isStarted) {
            CommandScheduler.run()
        }
    }
}