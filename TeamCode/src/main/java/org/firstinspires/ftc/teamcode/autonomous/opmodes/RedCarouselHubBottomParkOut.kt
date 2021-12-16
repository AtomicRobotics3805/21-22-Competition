package org.firstinspires.ftc.teamcode.autonomous.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.AutoRoutines
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetectionMB1220
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Autonomous(group = "Red", name = "Red Carousel Hub Bottom Park Out")
//@Disabled
class RedCarouselHubBottomParkOut: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = Constants.Color.RED
        TrajectoryFactory.initializeStartPositions()
        Constants.startPose = TrajectoryFactory.carouselStartPose

        OpModeController.initialize(this)

        CommandScheduler.commandsToSchedule += sequential {
            +ObjectDetectionMB1220.DetectCommand()
            +AutoRoutines.carouselHubBottomParkOutRoutine
        }
        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}