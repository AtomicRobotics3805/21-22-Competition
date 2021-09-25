package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Carousel
import org.firstinspires.ftc.teamcode.util.CommandGamepad
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

object Controls {
    private val gamepad1 = CommandGamepad(opMode.gamepad1)
    private val gamepad2 = CommandGamepad(opMode.gamepad2)

    fun registerGamepads() {
        CommandScheduler.registerGamepads(gamepad1, gamepad2)
    }

    fun registerCommands() {
        CommandScheduler.commands += MecanumDrive.driverControlled(opMode.gamepad1)

        gamepad1.a.pressed.command = { MecanumDrive.switchSpeed }
        gamepad1.x.pressed.command = { Carousel.start }
        gamepad1.x.released.command = { Carousel.stop }
        gamepad1.y.pressed.command = { Carousel.switch }
    }
}