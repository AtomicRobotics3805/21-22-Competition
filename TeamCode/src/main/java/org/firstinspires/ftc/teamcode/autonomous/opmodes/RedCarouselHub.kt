package org.firstinspires.ftc.teamcode.autonomous.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.AutoRoutines
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Autonomous(group = "Red", name = "Red Carousel Hub Front")
@Disabled
class RedCarouselHub: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = Constants.Color.RED
        TrajectoryFactory.initializeStartPositions()
        Constants.startPose = TrajectoryFactory.carouselStartPose

        OpModeController.initialize(this)

        CommandScheduler.commandsToSchedule += AutoRoutines.carouselHubRoutine

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}