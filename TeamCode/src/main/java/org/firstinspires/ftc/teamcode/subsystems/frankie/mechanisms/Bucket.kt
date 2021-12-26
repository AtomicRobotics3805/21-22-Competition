package org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.TimedCustomCommand
import kotlin.math.abs

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Bucket {

    fun initialize() {
        Latch.initialize()
        Rotator.initialize()
    }

    object Latch {
        enum class Position {
            OPEN,
            CLOSE
        }

        @JvmField
        var LATCH_NAME = "latch"

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
        private lateinit var latchServo: Servo

        fun initialize() {
            latchServo = opMode.hardwareMap.get(Servo::class.java, LATCH_NAME)
        }

        fun moveServo(position: Double, state: Position) =
            TimedCustomCommand(time = abs(position - latchServo.position),
                _start = {
                    latchServo.position = position
                    this.position = state
                })
    }

    object Rotator {
        enum class Position {
            COLLECT,
            DOWN
        }

        @JvmField
        var ROTATOR_NAME = "bucketRotator"

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
        private lateinit var bucketRotatorServo: Servo

        fun initialize() {
            bucketRotatorServo = opMode.hardwareMap.get(Servo::class.java, ROTATOR_NAME)
        }

        fun moveServo(position: Double, state: Position) =
            TimedCustomCommand(time = abs(position - bucketRotatorServo.position),
                _start = {
                    bucketRotatorServo.position = position
                    this.position = state
                })
    }
}