package org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
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
            SCORE,
            COLLECT,
            LOCK
        }

        @JvmField
        var LATCH_NAME = "bucketLockServo"

        @JvmField
        var SCORE_POSITION = 0.3
        @JvmField
        var COLLECT_POSITION = 0.4
        @JvmField
        var LOCK_POSITION = 0.65

        val open: AtomicCommand
            get() = moveServo(SCORE_POSITION, Position.SCORE)
        val collect: AtomicCommand
            get() = moveServo(COLLECT_POSITION, Position.COLLECT)
        val close: AtomicCommand
            get() = moveServo(LOCK_POSITION, Position.LOCK)
        val switch: AtomicCommand
            get() = if (position == Position.SCORE) close else open


        var position = Position.SCORE
        private lateinit var latchServo: Servo

        fun initialize() {
            latchServo = opMode.hardwareMap.get(Servo::class.java, LATCH_NAME)
        }

        fun moveServo(position: Double, state: Position) =
            /*Timed*/CustomCommand(/*time = abs(position - latchServo.position),*/
                _start = {
                    latchServo.position = position
                    this.position = state
                })
    }

    object Rotator {
        enum class Position {
            IDLE,
            COLLECT,
            SCORE
        }

        @JvmField
        var ROTATOR_NAME = "bucketRotateServo"

        @JvmField
        var SCORE_POSITION = 0.5
        @JvmField
        var COLLECT_POSITION = 0.0

        val score: AtomicCommand
            get() = moveServo(SCORE_POSITION, Position.SCORE)
        val collect: AtomicCommand
            get() = moveServo(COLLECT_POSITION, Position.COLLECT)
        val switch: AtomicCommand
            get() = if (position == Position.COLLECT) score else collect


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