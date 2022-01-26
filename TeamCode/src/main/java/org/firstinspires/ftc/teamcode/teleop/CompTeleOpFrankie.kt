package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Carousel
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Lift
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler
import org.firstinspires.ftc.teamcode.util.commands.other.TelemetryCommand

@Suppress("unused")
@TeleOp(name = "Competition Frankie TeleOp")
class CompTeleOpFrankie: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this

        MecanumDrive.initialize()
        Bucket.initialize()
        Intake.initialize()
        Lift.initialize(resetPosition = false)
        Carousel.initialize()
        TrajectoryFactory.initializeStartPositions()
        Controls.registerGamepads()
        CommandScheduler.cancelAll()

        waitForStart()

        Controls.registerFrankieCompetitionCommands()

        while (opModeIsActive()) {
            CommandScheduler.run()
            telemetry.update()
        }
    }
}