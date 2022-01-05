package org.firstinspires.ftc.teamcode.autonomous.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.TrioAutoRoutines
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Autonomous
class TestRoutine : LinearOpMode() {

    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = Constants.Color.BLUE
        TrajectoryFactory.initializeStartPositions()
        Constants.startPose = Pose2d()

        MecanumDrive.initialize()
        TrajectoryFactory.initializeTrioTrajectories()

        CommandScheduler.cancelAll()

        while (!isStarted) {
            CommandScheduler.run()
        }

        CommandScheduler.commandsToSchedule += TrioAutoRoutines.testRoutine

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}