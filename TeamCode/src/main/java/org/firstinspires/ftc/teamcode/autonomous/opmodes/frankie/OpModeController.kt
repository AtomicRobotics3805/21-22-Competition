package org.firstinspires.ftc.teamcode.autonomous.opmodes.frankie

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetectionMB1220
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Carousel
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Lift
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

object OpModeController {
    fun initialize() {
        Bucket.initialize()
        Carousel.initialize()
        Intake.initialize()
        Lift.initialize()
        ObjectDetectionMB1220.initialize()
        TrajectoryFactory.initializeTrioTrajectories()

        CommandScheduler.cancelAll()

        if (Constants.opMode is LinearOpMode) {
            while (!(Constants.opMode as LinearOpMode).isStarted) {
                CommandScheduler.run()
            }
        }
    }
}