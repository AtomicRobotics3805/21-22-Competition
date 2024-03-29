package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.other.TimedCustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import kotlin.math.abs

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object BucketLock : Subsystem {

    enum class Position {
        OPEN,
        CLOSE
    }

    @JvmField
    var LATCH_NAME = "lock"
    @JvmField
    var OPEN_POSITION = 0.45
    @JvmField
    var CLOSE_POSITION = 0.1

    val open: AtomicCommand
        get() = moveServo(OPEN_POSITION, Position.OPEN)
    val close: AtomicCommand
        get() = moveServo(CLOSE_POSITION, Position.CLOSE)
    val switch: AtomicCommand
        get() = if (position == Position.OPEN) close else open


    var position = Position.OPEN
    private lateinit var lockServo: Servo

    fun initialize() {
        lockServo = opMode.hardwareMap.get(Servo::class.java, LATCH_NAME)
    }

    fun moveServo(position: Double, state: Position) =
            TimedCustomCommand(time = abs(position - lockServo.position),
                    _start = {
                        lockServo.position = position
                        this.position = state
                    })
}