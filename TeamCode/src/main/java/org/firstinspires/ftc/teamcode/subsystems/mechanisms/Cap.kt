package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.TimedCustomCommand
import kotlin.math.abs

@Config
object Cap {
    @JvmField
    var CAP_NAME = "cap"
    @JvmField
    var DOWN_POSITION = 0.22
    @JvmField
    var UP_POSITION = 0.7

    enum class Position {
        UP,
        DOWN
    }

    val down: AtomicCommand
        get() = moveServo(DOWN_POSITION, Position.DOWN)
    val up: AtomicCommand
        get() = moveServo(UP_POSITION, Position.UP)
    val switch: AtomicCommand
        get() = if(position == Position.UP) down else up

    var position = Position.DOWN
    private lateinit var servo: Servo

    fun initialize() {
        servo = Constants.opMode.hardwareMap.get(Servo::class.java, CAP_NAME)
    }

    fun moveServo(position: Double, state: Position) =
        TimedCustomCommand(time = abs(position - servo.position),
            _start = {
                servo.position = position
                this.position = state
            })
}