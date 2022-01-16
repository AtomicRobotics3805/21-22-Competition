package org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.other.TimedCustomCommand
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Config
object OdometryServo {
    @JvmField
    var DEAD_WHEEL_SERVO_NAME = "deadWheelServo"

    @JvmField
    var DOWN_POSITION = 0.0
    @JvmField
    var UP_POSITION = 0.3

    enum class Position {
        UP,
        DOWN
    }

    val down: AtomicCommand
        get() = moveServo(DOWN_POSITION, Position.DOWN)
    val up: AtomicCommand
        get() = moveServo(UP_POSITION, Position.UP)
    val switch: AtomicCommand
        get() = if (position == Position.UP) down else up
    val upAutonomous: AtomicCommand
        get() = sequential {
            +StopTranslationalPIDCommand()
            +up
        }
    val resetTranslationalPID: AtomicCommand
        get() = ResetTranslationalPIDCommand()

    var position = Position.DOWN
    private lateinit var odometryServo: Servo

    fun initialize() {
        odometryServo = Constants.opMode.hardwareMap.get(Servo::class.java, DEAD_WHEEL_SERVO_NAME)
    }

    fun moveServo(position: Double, state: Position) =
        TimedCustomCommand(time = 0.0,//abs(position - odometryServo.position),
            _start = {
                odometryServo.position = position
                OdometryServo.position = state
            })

    class StopTranslationalPIDCommand : AtomicCommand() {
        override fun start() {
            MecanumDrive.TRANSLATIONAL_PID = PIDCoefficients(0.0, 0.0, 0.0)
            MecanumDrive.follower = HolonomicPIDVAFollower(
                MecanumDrive.TRANSLATIONAL_PID, MecanumDrive.TRANSLATIONAL_PID, MecanumDrive.HEADING_PID,
                Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5)
        }
    }

    class ResetTranslationalPIDCommand : AtomicCommand() {
        override fun start() {
            MecanumDrive.TRANSLATIONAL_PID = MecanumDrive.STARTING_TRANSLATIONAL_PID
            MecanumDrive.follower = HolonomicPIDVAFollower(
                MecanumDrive.TRANSLATIONAL_PID, MecanumDrive.TRANSLATIONAL_PID, MecanumDrive.HEADING_PID,
                Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5)
        }
    }
}