package org.firstinspires.ftc.teamcode.autonomous.opmodes.frankie

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.FrankieAutoRoutines
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

class FrankieBlueFreights : LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = Constants.Color.BLUE
        TrajectoryFactory.initializeStartPositions()
        Constants.startPose = TrajectoryFactory.frankieStartPose

        OpModeController.initialize()

        CommandScheduler.commandsToSchedule += FrankieAutoRoutines.noCarouselFreightRoutine

        while (opModeIsActive() && CommandScheduler.commandsToSchedule.isNotEmpty()) {
            CommandScheduler.run()
        }
    }

}