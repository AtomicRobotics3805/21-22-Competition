package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Arm
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Carousel
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Suppress("unused")
@TeleOp(name = "Competition Testing")
class CompTeleOp: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this

        MecanumDrive.initialize()
        Arm.initialize()
        Bucket.initialize()
        Carousel.initialize()
        Intake.initialize()
        Controls.registerGamepads()
        CommandScheduler.registerSubsystems(MecanumDrive, Arm, Bucket, Carousel, Intake)

        waitForStart()

        Controls.registerCommands()

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}