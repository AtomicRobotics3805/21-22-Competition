package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Carousel
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Lift
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Suppress("unused")
@TeleOp(name = "Testing Frankie TeleOp")
class TestingTeleOpFrankie: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this

        Bucket.initialize()
        Carousel.initialize()
        Intake.initialize()
        Lift.initialize()
        Controls.registerGamepads()
        CommandScheduler.cancelAll()

        waitForStart()

        Controls.registerFrankieTestingCommands()

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}