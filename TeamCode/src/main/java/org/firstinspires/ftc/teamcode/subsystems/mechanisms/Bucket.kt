package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.TimedCustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import kotlin.math.abs

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Bucket : Subsystem {

    enum class Position {
        UP,
        DOWN
    }

    @JvmField
    var BUCKET_NAME = "bucket"
    @JvmField
    var DROP_POSITION = 0.0
    @JvmField
    var UP_POSITION = 0.95

    val drop: AtomicCommand
        get() = moveServo(DROP_POSITION, Position.DOWN)
    val up: AtomicCommand
        get() = moveServo(UP_POSITION, Position.UP)
    val switch: AtomicCommand
        get() = if (position == Position.UP) drop else up


    var position = Position.UP
    private lateinit var servo: Servo

    fun initialize() {
        servo = opMode.hardwareMap.get(Servo::class.java, BUCKET_NAME)
    }

    fun moveServo(position: Double, state: Position) =
            TimedCustomCommand(time = abs(position - servo.position),
                    _start = {
                        servo.position = position
                        this.position = state
                    })
}