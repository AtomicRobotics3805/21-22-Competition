package org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.other.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.other.TimedCustomCommand
import kotlin.math.abs

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Bucket {

    fun initialize() {
        Lock.initialize()
        Rotator.initialize()
    }

    object Lock {
        enum class Position {
            SCORE,
            COLLECT,
            LOCK
        }

        @JvmField
        var LOCK_NAME = "bucketLockServo"

        @JvmField
        var SCORE_POSITION = 0.3
        @JvmField
        var COLLECT_POSITION = 0.45
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
        private lateinit var lockServo: Servo

        fun initialize() {
            lockServo = opMode.hardwareMap.get(Servo::class.java, LOCK_NAME)
        }

        fun moveServo(position: Double, state: Position) =
            CustomCommand(
                _start = {
                    lockServo.position = position
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
        var COLLECT_POSITION = 0.05

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