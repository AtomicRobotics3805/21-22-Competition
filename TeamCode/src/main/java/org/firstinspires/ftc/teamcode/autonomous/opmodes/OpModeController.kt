package org.firstinspires.ftc.teamcode.autonomous.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetectionMB1220
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.*
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
        ContainerSensor.initialize()
        OdometryServo.initialize()
        TrajectoryFactory.initializeTrajectories()

        CommandScheduler.registerSubsystems(MecanumDrive, Arm, Bucket, Carousel, Intake)
        CommandScheduler.cancelAll()

        CommandScheduler.commandsToSchedule += OdometryServo.resetTranslationalPID

        while (!opMode.isStarted) {
            CommandScheduler.run()
            opMode.telemetry.addData("Arm Coefficients", Arm.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION))
            opMode.telemetry.update()
        }
    }
}