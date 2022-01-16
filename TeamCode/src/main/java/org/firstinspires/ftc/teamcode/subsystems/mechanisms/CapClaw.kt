package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.other.TimedCustomCommand
import kotlin.math.abs

@Config
object CapClaw {
    @JvmField
    var CAP_CLAW_NAME = "claw"
    @JvmField
    var OPEN_POSITION = 0.0
    @JvmField
    var CLOSE_POSITION = 0.5

    enum class Position {
        OPEN,
        CLOSED
    }

    val open: AtomicCommand
        get() = moveServo(OPEN_POSITION, Position.OPEN)
    val close: AtomicCommand
        get() = moveServo(CLOSE_POSITION, Position.CLOSED)
    val switch: AtomicCommand
        get() = if(position == Position.OPEN) close else open

    var position = Position.OPEN
    private lateinit var capServo: Servo

    fun initialize() {
        capServo = Constants.opMode.hardwareMap.get(Servo::class.java, CAP_CLAW_NAME)
    }

    fun moveServo(position: Double, state: Position) =
        TimedCustomCommand(time = abs(position - capServo.position),
            _start = {
                capServo.position = position
                this.position = state
            })
}