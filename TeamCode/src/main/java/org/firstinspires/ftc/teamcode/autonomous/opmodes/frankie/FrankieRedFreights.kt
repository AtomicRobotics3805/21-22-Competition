package org.firstinspires.ftc.teamcode.autonomous.opmodes.frankie

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.autonomous.FrankieRoutines
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Lift
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Autonomous(name="Frankie Red Freights Auto")
class FrankieRedFreights : LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = Constants.Color.RED
        TrajectoryFactory.initializeStartPositions()
        Constants.startPose = TrajectoryFactory.outsideWarehouseStartPose

        OpModeController.initialize()

        CommandScheduler.scheduleCommand(FrankieRoutines.noCarouselFreightRoutine)

        while (opModeIsActive() && (CommandScheduler.commandsToSchedule.isNotEmpty() || CommandScheduler.commands.isNotEmpty())) {
            CommandScheduler.run()
            telemetry.addData("Lift Swivel Encoder Position", Lift.Swivel.swivelMotor.currentPosition)
            telemetry.addData("Lift Pivot Angle", Lift.Pivot.angle)
            telemetry.addData("Lift Extension Encoder Position", Lift.Extender.extensionMotor.currentPosition)
            telemetry.update()
        }

        Lift.saveOffsets()
    }

}