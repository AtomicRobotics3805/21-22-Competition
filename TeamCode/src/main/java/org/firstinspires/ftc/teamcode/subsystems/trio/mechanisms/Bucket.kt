package org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.TimedCustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import kotlin.math.abs

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Bucket : Subsystem {

    enum class Position {
        COLLECT,
        DOWN
    }

    @JvmField
    var BUCKET_NAME = "bucket"
    @JvmField
    var DROP_POSITION = 0.25
    @JvmField
    var COLLECT_POSITION = 0.9

    val drop: AtomicCommand
        get() = moveServo(DROP_POSITION, Position.DOWN)
    val up: AtomicCommand
        get() = moveServo(COLLECT_POSITION, Position.COLLECT)
    val switch: AtomicCommand
        get() = if (position == Position.COLLECT) drop else up


    var position = Position.COLLECT
    private lateinit var bucketServo: Servo

    fun initialize() {
        bucketServo = opMode.hardwareMap.get(Servo::class.java, BUCKET_NAME)
    }

    fun moveServo(position: Double, state: Position) =
            TimedCustomCommand(time = abs(position - bucketServo.position),
                    _start = {
                        bucketServo.position = position
                        Bucket.position = state
                    })
}