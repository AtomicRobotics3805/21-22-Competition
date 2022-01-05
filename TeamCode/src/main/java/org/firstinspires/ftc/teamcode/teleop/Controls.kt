package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.autonomous.FrankieRoutines
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Carousel
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Lift
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

    fun registerFrankieCompetitionCommands() {
        CommandScheduler.commandsToSchedule += MecanumDrive.driverControlled(opMode.gamepad1)

        gamepad1.a.pressed.command = { MecanumDrive.switchSpeed }
        gamepad1.x.pressed.command = { FrankieRoutines.dropAndCollectTeleOpRoutine }
        gamepad1.y.pressed.command = { FrankieRoutines.transferMoveLiftTeleOpRoutine }

        gamepad2.a.pressed.command = { FrankieRoutines.dropAndCollectTeleOpRoutine }
        gamepad2.b.pressed.command = { FrankieRoutines.transferMoveLiftTeleOpRoutine }
        gamepad2.x.pressed.command = { Intake.Extender.switch }
        gamepad2.y.pressed.command = { Intake.Lock.switch }
        gamepad2.leftBumper.pressed.command = { Intake.Spinner.switch }
        gamepad1.rightBumper.pressed.command = { Intake.Pushthrough.push }
        gamepad1.rightBumper.released.command = { Intake.Pushthrough.idle }
        //gamepad2.leftTrigger.pressed.command = { Carousel.powerCarouselTrigger(gamepad2.gamepad.leftTrigger) }
        gamepad2.rightTrigger.pressed.command = { Bucket.Rotator.switch }
        gamepad2.dpadUp.pressed.command = { Lift.Swivel.manualUp }
        gamepad2.dpadUp.released.command = { Lift.Swivel.idle }
        gamepad2.dpadDown.pressed.command = { Lift.Swivel.manualDown }
        gamepad2.dpadDown.released.command = { Lift.Swivel.idle }
        gamepad2.dpadLeft.pressed.command = { Lift.Pivot.manualLeft }
        gamepad2.dpadLeft.released.command = { Lift.Pivot.idle }
        gamepad2.dpadRight.pressed.command = { Lift.Pivot.manualRight }
        gamepad2.dpadRight.released.command = { Lift.Pivot.idle }
    }

    fun registerFrankieTestingCommands() {
        CommandScheduler.commandsToSchedule += MecanumDrive.driverControlled(opMode.gamepad1)

        gamepad1.a.pressed.command = { Intake.Extender.extend }
        gamepad1.b.pressed.command = { Intake.Extender.retract }
        gamepad1.x.pressed.command = { Intake.Lock.open }
        gamepad1.y.pressed.command = { Intake.Lock.close }
        gamepad1.rightTrigger.pressed.command = { Intake.Spinner.start }
        gamepad1.rightTrigger.released.command = { Intake.Spinner.stop }
        //gamepad1.leftBumper.pressed.command = { Intake.Rotator.down }
        gamepad1.rightBumper.pressed.command = { Intake.Pushthrough.push }
        gamepad1.rightBumper.released.command = { Intake.Pushthrough.idle }
        //gamepad1.leftTrigger.pressed.command = { Carousel.powerCarouselTrigger(gamepad1.gamepad.leftTrigger) }

        gamepad2.a.pressed.command = { Lift.Extender.fullExtend }
        gamepad2.b.pressed.command = { Lift.Extender.retract }
        gamepad2.x.pressed.command = { Bucket.Latch.open }
        gamepad2.y.pressed.command = { Bucket.Latch.close }
        gamepad2.leftBumper.pressed.command = { Bucket.Rotator.drop }
        gamepad2.rightBumper.pressed.command = { Bucket.Rotator.up }
        gamepad2.rightTrigger.pressed.command = { Bucket.Rotator.switch }
        gamepad2.dpadUp.pressed.command = { Lift.Swivel.manualUp }
        gamepad2.dpadUp.released.command = { Lift.Swivel.idle }
        gamepad2.dpadDown.pressed.command = { Lift.Swivel.ToCollectCareful() }
        gamepad2.dpadLeft.pressed.command = { Lift.Pivot.manualLeft }
        gamepad2.dpadLeft.released.command = { Lift.Pivot.idle }
        gamepad2.dpadRight.pressed.command = { Lift.Pivot.manualRight }
        gamepad2.dpadRight.released.command = { Lift.Pivot.idle }
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