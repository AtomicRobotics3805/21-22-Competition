package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Carousel
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Lift
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Suppress("unused")
@TeleOp(name = "Testing Frankie TeleOp")
class TestingTeleOpFrankie: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this

        MecanumDrive.initialize()
        Bucket.initialize()
        Intake.initialize()
        Lift.initialize()
        TrajectoryFactory.initializeStartPositions()
        Controls.registerGamepads()
        CommandScheduler.cancelAll()

        waitForStart()

        Controls.registerFrankieTestingCommands()

        while (opModeIsActive()) {
            CommandScheduler.run()
            telemetry.addData("Lift Extension Position", Lift.Extender.extensionMotor.currentPosition.toDouble() / Lift.Extender.EXTENDER_TICKS_PER_INCH)
            telemetry.addData("Lift Pivot Position", Lift.Pivot.liftPivotMotor.currentPosition)
            telemetry.addData("Lift Swivel Position", Lift.Swivel.swivelMotor.currentPosition)
            telemetry.addData("Lift Swivel Height", Lift.Swivel.swivelMotor.currentPosition / Lift.Swivel.SWIVEL_TICKS_PER_DEGREE)
            telemetry.addData("Coefficients", Lift.Pivot.liftPivotMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION))
            telemetry.update()
        }
    }
}