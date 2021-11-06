package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.TimedCustomCommand
import kotlin.math.abs

@Config
object DeadWheelServo {
    @JvmField
    var DEAD_WHEEL_SERVO_NAME = "deadWheelServo"

    @JvmField
    var DOWN_POSITION = 0.0
    @JvmField
    var UP_POSITION = 1.0

    enum class Position {
        UP,
        DOWN
    }

    val down: AtomicCommand
        get() = moveServo(DOWN_POSITION, Position.DOWN)
    val up: AtomicCommand
        get() = moveServo(UP_POSITION, Position.UP)
    val switch: AtomicCommand
        get() = if (Bucket.position == Bucket.Position.UP) down else up

    var position = Position.DOWN
    private lateinit var servo: Servo

    fun initialize() {
        servo = Constants.opMode.hardwareMap.get(Servo::class.java, DEAD_WHEEL_SERVO_NAME)
    }

    fun moveServo(position: Double, state: Position) =
        TimedCustomCommand(time = abs(position - servo.position),
            _start = {
                servo.position = position
                this.position = state
            })
}