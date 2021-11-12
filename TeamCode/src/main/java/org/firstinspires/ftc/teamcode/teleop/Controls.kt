package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.*
import org.firstinspires.ftc.teamcode.util.CommandGamepad
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler
import org.firstinspires.ftc.teamcode.util.commands.sequential

object Controls {
    val gamepad1 = CommandGamepad(opMode.gamepad1)
    val gamepad2 = CommandGamepad(opMode.gamepad2)

    fun registerGamepads() {
        CommandScheduler.registerGamepads(gamepad1, gamepad2)
    }

    fun registerCommands() {
        CommandScheduler.commandsToSchedule += MecanumDrive.driverControlled(opMode.gamepad1)

        gamepad1.a.pressed.command = { MecanumDrive.switchSpeed }
        gamepad1.x.pressed.command = { Carousel.switch }
        gamepad1.y.pressed.command = { Bucket.switch }
        gamepad1.b.pressed.command = { Intake.switch }
        gamepad1.dpadUp.pressed.command = { Arm.start }
        gamepad1.dpadUp.released.command = { Arm.stop }
        gamepad1.dpadDown.pressed.command = { Arm.reverse }
        gamepad1.dpadDown.released.command = { Arm.stop }
        gamepad1.dpadRight.pressed.command = { Arm.toHigh }
        gamepad1.dpadLeft.pressed.command = { Arm.toStart }
        gamepad1.leftBumper.pressed.command = { sequential {
            +Arm.toHigh
            +Bucket.drop
            +Arm.toStart
            +Bucket.up
        } }
        gamepad1.rightBumper.pressed.command = { DeadWheelServo.switch }
        gamepad1.leftBumper.pressed.command = { Cap.switch }
        gamepad2.dpadRight.pressed.command = { Arm.toHigh }
        gamepad2.dpadUp.pressed.command = { Arm.toMedium }
        gamepad2.dpadLeft.pressed.command = { Arm.toLow }
        gamepad2.dpadDown.pressed.command = { Arm.toStart }
        gamepad2.x.pressed.command = { Bucket.up }
        gamepad2.y.pressed.command = { Bucket.drop }
        gamepad2.a.pressed.command = { BucketLatch.close }
        gamepad2.b.pressed.command = { BucketLatch.open }
    }
}