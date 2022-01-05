package org.firstinspires.ftc.teamcode.autonomous.opmodes.trio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.TrioAutoRoutines
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Autonomous(group = "Blue", name = "Blue Carousel Hub Front")
@Disabled
class BlueCarouselHub: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = Constants.Color.BLUE
        TrajectoryFactory.initializeStartPositions()
        Constants.startPose = TrajectoryFactory.carouselStartPose

        OpModeController.initialize(this)

        CommandScheduler.commandsToSchedule += TrioAutoRoutines.carouselHubRoutine

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}