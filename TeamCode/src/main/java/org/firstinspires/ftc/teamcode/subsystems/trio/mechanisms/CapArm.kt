package org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.TimedCustomCommand
import kotlin.math.abs

@Config
object CapArm {
    @JvmField
    var CAP_NAME = "cap"
    @JvmField
    var DOWN_POSITION = 0.0
    @JvmField
    var UP_POSITION = 0.875
    @JvmField
    var IDLE_POSITION = 0.7

    enum class Position {
        UP,
        DOWN,
        IDLE
    }

    val down: AtomicCommand
        get() = moveServo(DOWN_POSITION, Position.DOWN)
    val up: AtomicCommand
        get() = moveServo(UP_POSITION, Position.UP)
    val idle: AtomicCommand
        get() = moveServo(IDLE_POSITION, Position.IDLE)
    val switch: AtomicCommand
        get() = if(position == Position.UP) down else up

    var position = Position.DOWN
    private lateinit var capServo: Servo

    fun initialize() {
        capServo = Constants.opMode.hardwareMap.get(Servo::class.java, CAP_NAME)
    }

    fun moveServo(position: Double, state: Position) =
        TimedCustomCommand(time = abs(position - capServo.position),
            _start = {
                capServo.position = position
                CapArm.position = state
            })
}