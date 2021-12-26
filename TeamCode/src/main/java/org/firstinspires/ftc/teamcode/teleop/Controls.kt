package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.subsystems.trio.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.CommandGamepad
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler
import org.firstinspires.ftc.teamcode.util.commands.sequential
import org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms.Carousel as TrioCarousel
import org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms.Bucket as TrioBucket
import org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms.BucketLock as TrioBucketLock
import org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms.Intake as TrioIntake
import org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms.OdometryServo as TrioOdometryServo
import org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms.Arm as TrioArm
import org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms.CapArm as TrioCapArm

object Controls {
    val gamepad1 = CommandGamepad(opMode.gamepad1)
    val gamepad2 = CommandGamepad(opMode.gamepad2)

    fun registerGamepads() {
        CommandScheduler.registerGamepads(gamepad1, gamepad2)
    }

    fun registerFrankieCommands() {
        CommandScheduler.commandsToSchedule += MecanumDrive.driverControlled(opMode.gamepad1)

        gamepad1.a.pressed.command = { MecanumDrive.switchSpeed }
        gamepad1.x.pressed.command = { TrioCarousel.switch }
        gamepad1.y.pressed.command = { TrioBucket.switch }
    }

    fun registerTrioCommands() {
        CommandScheduler.commandsToSchedule += MecanumDrive.driverControlled(opMode.gamepad1)

        gamepad1.a.pressed.command = { MecanumDrive.switchSpeed }
        gamepad1.x.pressed.command = { TrioCarousel.switch }
        gamepad1.y.pressed.command = { TrioBucket.switch }
        gamepad1.b.pressed.command = { TrioIntake.switch }
        gamepad1.dpadUp.pressed.command = { TrioArm.start }
        gamepad1.dpadUp.released.command = { TrioArm.stop }
        gamepad1.dpadDown.pressed.command = { TrioArm.reverse }
        gamepad1.dpadDown.released.command = { TrioArm.stop }
        gamepad1.dpadRight.pressed.command = { TrioArm.toHigh }
        gamepad1.dpadLeft.pressed.command = { TrioArm.toStart }
        gamepad1.leftBumper.pressed.command = { sequential {
            +TrioArm.toHigh
            +TrioBucket.drop
            +TrioArm.toStart
            +TrioBucket.up
        } }
        gamepad1.rightBumper.pressed.command = { TrioOdometryServo.switch }
        gamepad1.leftBumper.pressed.command = { TrioCapArm.switch }
        gamepad2.dpadRight.pressed.command = { TrioArm.toHigh }
        gamepad2.dpadUp.pressed.command = { TrioArm.toMedium }
        gamepad2.dpadLeft.pressed.command = { TrioArm.toLow }
        gamepad2.dpadDown.pressed.command = { TrioArm.toStart }
        gamepad2.x.pressed.command = { TrioBucket.up }
        gamepad2.y.pressed.command = { TrioBucket.drop }
        gamepad2.a.pressed.command = { TrioBucketLock.close }
        gamepad2.b.pressed.command = { TrioBucketLock.open }
        gamepad2.leftBumper.pressed.command = { TrioCarousel.fullRotation }
        gamepad2.rightBumper.pressed.command = { TrioCarousel.fullRotationReverse }
    }
}