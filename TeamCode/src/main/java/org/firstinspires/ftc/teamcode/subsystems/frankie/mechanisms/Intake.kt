package org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.TimedCustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import kotlin.math.abs
import kotlin.math.max

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Intake {
    fun initialize() {
        Extender.initialize()
        Rotator.initialize()
        Spinner.initialize()
        Lock.initialize()
        ColorSensor.initialize()
    }

    @Config
    object Extender : Subsystem {
        @JvmField
        var EXTENDED_POSITION_LEFT = 0.0
        @JvmField
        var EXTENDED_POSITION_RIGHT = 1.0
        @JvmField
        var RETRACTED_POSITION_LEFT = 1.0
        @JvmField
        var RETRACTED_POSITION_RIGHT = 0.0
        @JvmField
        var LEFT_EXTENDER_NAME = "leftIntakeExtender"
        @JvmField
        var RIGHT_EXTENDER_NAME = "rightIntakeExtender"

        private var extended = false
        private lateinit var leftExtensionServo: Servo
        private lateinit var rightExtensionServo: Servo

        fun initialize() {
            leftExtensionServo = opMode.hardwareMap.get(Servo::class.java, LEFT_EXTENDER_NAME)
            rightExtensionServo = opMode.hardwareMap.get(Servo::class.java, RIGHT_EXTENDER_NAME)
        }

        val extend: AtomicCommand
            get() = TimedCustomCommand(
                time = max(abs(leftExtensionServo.position - EXTENDED_POSITION_LEFT),
                    abs(rightExtensionServo.position - EXTENDED_POSITION_RIGHT)),
                _start = {
                    extended = true
                    leftExtensionServo.position = EXTENDED_POSITION_LEFT
                    rightExtensionServo.position = EXTENDED_POSITION_RIGHT
                })
        val retract: AtomicCommand
            get() = TimedCustomCommand(
                time = max(abs(leftExtensionServo.position - RETRACTED_POSITION_LEFT),
                    abs(rightExtensionServo.position - RETRACTED_POSITION_RIGHT)),
                _start = {
                    extended = false
                    leftExtensionServo.position = RETRACTED_POSITION_LEFT
                    rightExtensionServo.position = RETRACTED_POSITION_RIGHT
                })
        val switch: AtomicCommand
            get() = if (extended) retract else extend
    }

    @Config
    object Rotator : Subsystem {
        @JvmField
        var UP_POSITION_LEFT = 0.0
        @JvmField
        var UP_POSITION_RIGHT = 1.0
        @JvmField
        var DOWN_POSITION_LEFT = 0.2
        @JvmField
        var DOWN_POSITION_RIGHT = 0.8
        @JvmField
        var LEFT_ROTATOR_NAME = "leftIntakeRotator"
        @JvmField
        var RIGHT_ROTATOR_NAME = "rightIntakeRotator"

        private var rotatedUp = false
        private lateinit var leftRotatorServo: Servo
        private lateinit var rightRotatorServo: Servo

        fun initialize() {
            leftRotatorServo = opMode.hardwareMap.get(Servo::class.java, LEFT_ROTATOR_NAME)
            rightRotatorServo = opMode.hardwareMap.get(Servo::class.java, RIGHT_ROTATOR_NAME)
        }

        val up: AtomicCommand
            get() = moveServos(UP_POSITION_LEFT, UP_POSITION_RIGHT, true)
        val down: AtomicCommand
            get() = moveServos(DOWN_POSITION_LEFT, DOWN_POSITION_RIGHT, false)
        val switch: AtomicCommand
            get() = if (rotatedUp) down else up
        val downStart: AtomicCommand
            get() = moveServos(DOWN_POSITION_LEFT, DOWN_POSITION_RIGHT, false, 0.3)

        fun moveServos(leftPosition: Double,
                       rightPosition: Double,
                       _rotatedUp: Boolean,
                       time: Double = max(abs(leftRotatorServo.position - leftPosition),
                           abs(rightRotatorServo.position - rightPosition))): AtomicCommand =
            TimedCustomCommand(time = time, _start = {
                rotatedUp = _rotatedUp
                leftRotatorServo.position = leftPosition
                rightRotatorServo.position = rightPosition
        })
    }

    @Config
    object Spinner : Subsystem {
        @JvmField
        var SPINNER_INTAKING_SPEED = 1.0
        @JvmField
        var SPINNER_IDLE_SPEED = 0.1
        @JvmField
        var SPINNER_DIRECTION = DcMotorSimple.Direction.FORWARD
        @JvmField
        var SPINNER_COUNTS_PER_MOTOR_REV: Double = 28 * 3.7
        @JvmField
        var SPINNER_NAME = "intakeSpinner"

        private var spinning = false
        private lateinit var spinnerMotor: DcMotor

        fun initialize() {
            spinnerMotor = opMode.hardwareMap.get(DcMotor::class.java, SPINNER_NAME)
            spinnerMotor.direction = SPINNER_DIRECTION
        }

        val start: AtomicCommand
            get() = powerSpinner(SPINNER_INTAKING_SPEED, true)
        val idle: AtomicCommand
            get() = powerSpinner(SPINNER_IDLE_SPEED, false)
        val stop: AtomicCommand
            get() = powerSpinner(0.0, false)
        val switch: AtomicCommand
            get() = if (spinning) idle else start

        fun powerSpinner(power: Double, _spinning: Boolean) = CustomCommand(_start = {
            spinning = _spinning
            spinnerMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            spinnerMotor.power = power
        })
    }

    @Config
    object Lock : Subsystem {
        @JvmField
        var CLOSED_POSITION = 0.0
        @JvmField
        var OPEN_POSITION = 0.5
        @JvmField
        var LOCK_NAME = "intakeLock"

        private var closed = false
        private lateinit var lockServo: Servo

        fun initialize() {
            lockServo = opMode.hardwareMap.get(Servo::class.java, LOCK_NAME)
        }

        val close: AtomicCommand
            get() = moveServo(CLOSED_POSITION, true)
        val open: AtomicCommand
            get() = moveServo(OPEN_POSITION, false)
        val switch: AtomicCommand
            get() = if (closed) close else open

        fun moveServo(position: Double, _closed: Boolean): AtomicCommand = CustomCommand(_start = {
            lockServo.position = position
            closed = _closed
        })
    }

    @Config
    object ColorSensor {
        @JvmField
        var COLOR_SENSOR_NAME = "colorSensor"

        lateinit var colorSensor: RevColorSensorV3

        fun initialize() {
            colorSensor = opMode.hardwareMap.get(RevColorSensorV3::class.java, COLOR_SENSOR_NAME)
        }

        val hasFreight: Boolean
            get() = colorSensor.getDistance(DistanceUnit.INCH) != DistanceUnit.infinity
    }
}