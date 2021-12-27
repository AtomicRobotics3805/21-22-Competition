package org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetectionMB1220
import org.firstinspires.ftc.teamcode.subsystems.trio.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.MotorToPosition
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.round

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Lift {
    fun initialize() {
        Extender.initialize()
        Swivel.initialize()
        Pivot.initialize()
        LimitSwitch.initialize()
    }

    @Config
    object Extender : Subsystem {
        @JvmField
        var EXTENDER_NAME = "liftExtender"

        @JvmField
        var FULL_EXTENSION_DISTANCE = 30.0
        @JvmField
        var EXTENDER_SPEED = 1.0
        @JvmField
        var EXTENDER_DIRECTION = DcMotorSimple.Direction.FORWARD

        private const val PULLEY_DIAMETER = 1.0
        private const val PULLEY_CIRCUMFERENCE: Double = PULLEY_DIAMETER * PI
        private const val EXTENDER_TICKS_PER_REV: Double = 28 * 3.7
        private val EXTENDER_TICKS_PER_INCH: Int =
            round(EXTENDER_TICKS_PER_REV / PULLEY_CIRCUMFERENCE).toInt()

        private lateinit var extensionMotor: DcMotor

        fun initialize() {
            extensionMotor = opMode.hardwareMap.get(DcMotor::class.java, EXTENDER_NAME)
        }

        val fullExtend: AtomicCommand
            get() = ToPosition(FULL_EXTENSION_DISTANCE)
        //
        val fullExtendStopEarly: AtomicCommand
            get() = ToPosition(FULL_EXTENSION_DISTANCE, true)
        val retract: AtomicCommand
            get() = ToPosition(0.0)
        val manualUp: AtomicCommand
            get() = powerExtender(EXTENDER_SPEED)
        val manualDown: AtomicCommand
            get() = powerExtender(-EXTENDER_SPEED)
        val idle: AtomicCommand
            get() = CustomCommand(_start = {
                extensionMotor.power = EXTENDER_SPEED
                extensionMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                extensionMotor.targetPosition = extensionMotor.currentPosition
            })

        fun powerExtender(power: Double) = CustomCommand(_start = {
            extensionMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            extensionMotor.power = power
        })

        class ToPosition(val distance: Double, val stopEarly: Boolean = false) : MotorToPosition(extensionMotor, round(
            EXTENDER_TICKS_PER_INCH * distance).toInt() + extensionMotor.currentPosition, EXTENDER_SPEED) {
            override val _isDone: Boolean
                get() = if (!stopEarly) super._isDone else (distance * EXTENDER_TICKS_PER_REV) / error < 3
        }

        class ResetAtStart : AtomicCommand() {
            override val _isDone = false

            override fun start() {
                extensionMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
                extensionMotor.power = -EXTENDER_SPEED
            }

            override fun execute() {
                if (LimitSwitch.state) {
                    extensionMotor.power = 0.0
                    extensionMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    extensionMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
                    isDone = true
                }
            }
        }
    }

    @Config
    object Swivel {
        @JvmField
        var SWIVEL_NAME = "swivel"
        @JvmField
        var SWIVEL_SPEED = 1.0
        @JvmField
        var EXTENDER_DIRECTION = DcMotorSimple.Direction.FORWARD
        @JvmField
        var LOW_DEGREES = 80
        @JvmField
        var MIDDLE_DEGREES = 100
        @JvmField
        var HIGH_DEGREES = 120
        @JvmField
        var ACCEPTABLE_PIVOT_ANGLE = 5.0
        @JvmField
        var ACCEPTABLE_HEIGHT = 70
        

        private const val SWIVEL_GEAR_RATIO = 1.0
        private const val SWIVEL_TICKS_PER_REV: Double = 28 * 19.2
        private val SWIVEL_TICKS_PER_DEGREE: Int =
            round(SWIVEL_TICKS_PER_REV * SWIVEL_GEAR_RATIO / 360.0).toInt()
        private val LOW_POSITION: Int = SWIVEL_TICKS_PER_DEGREE * LOW_DEGREES
        private val MIDDLE_POSITION: Int = SWIVEL_TICKS_PER_DEGREE * MIDDLE_DEGREES
        private val HIGH_POSITION: Int = SWIVEL_TICKS_PER_DEGREE * HIGH_DEGREES

        private lateinit var swivelMotor: DcMotor

        fun initialize() {
            swivelMotor = opMode.hardwareMap.get(DcMotor::class.java, SWIVEL_NAME)
        }

        val toLow: AtomicCommand
            get() = MotorToPosition(swivelMotor, LOW_POSITION, SWIVEL_SPEED)
        val toMiddle: AtomicCommand
            get() = MotorToPosition(swivelMotor, MIDDLE_POSITION, SWIVEL_SPEED)
        val toHigh: AtomicCommand
            get() = MotorToPosition(swivelMotor, HIGH_POSITION, SWIVEL_SPEED)
        val manualUp: AtomicCommand
            get() = powerSwivel(SWIVEL_SPEED)
        val manualDown: AtomicCommand
            get() = powerSwivel(-SWIVEL_SPEED)
        val idle: AtomicCommand
            get() = CustomCommand(_start = {
                swivelMotor.power = SWIVEL_SPEED
                swivelMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                swivelMotor.targetPosition = swivelMotor.currentPosition
            })
        val toPreloadPosition: AtomicCommand
            get() = CustomCommand(
                getDone = { !swivelMotor.isBusy },
                _start = {
                    when (ObjectDetectionMB1220.position) {
                        ObjectDetectionMB1220.Position.LEFT -> swivelMotor.targetPosition = LOW_POSITION
                        ObjectDetectionMB1220.Position.MIDDLE -> swivelMotor.targetPosition = MIDDLE_POSITION
                        else -> swivelMotor.targetPosition = HIGH_POSITION
                    }
                    swivelMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    swivelMotor.power = SWIVEL_SPEED
                })

        fun powerSwivel(power: Double) = CustomCommand(_start = { swivelMotor.power = power})

        class ToCollectCareful : MotorToPosition(swivelMotor, 0, SWIVEL_SPEED) {
            override fun execute() {
                speed = if (abs(Pivot.angle) > ACCEPTABLE_PIVOT_ANGLE)
                    SWIVEL_SPEED
                else 0.0
                super.execute()
            }
        }
    }

    @Config
    object Pivot {
        @JvmField
        var PIVOT_NAME = "pivot"
        @JvmField
        var PIVOT_SPEED = 1.0
        @JvmField
        var EXTENDER_DIRECTION = DcMotorSimple.Direction.FORWARD

        private const val PIVOT_GEAR_RATIO = 1.0
        private const val PIVOT_TICKS_PER_REV: Double = 28 * 19.2
        private val PIVOT_TICKS_PER_DEGREE: Int =
            round(PIVOT_TICKS_PER_REV * PIVOT_GEAR_RATIO / 360.0).toInt()

        private lateinit var liftPivotMotor: DcMotor

        fun initialize() {
            liftPivotMotor = opMode.hardwareMap.get(DcMotor::class.java, PIVOT_NAME)
        }

        val angle: Double
            get() = liftPivotMotor.currentPosition / PIVOT_TICKS_PER_DEGREE.toDouble()
        val manualLeft: AtomicCommand
            get() = powerPivot(-PIVOT_SPEED)
        val manualRight: AtomicCommand
            get() = powerPivot(PIVOT_SPEED)
        val idle: AtomicCommand
            get() = CustomCommand(_start = {
                liftPivotMotor.power = PIVOT_SPEED
                liftPivotMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                liftPivotMotor.targetPosition = liftPivotMotor.currentPosition
            })

        fun powerPivot(power: Double) = CustomCommand(_start = { liftPivotMotor.power = power})
        fun toAngle(angle: Double): AtomicCommand =
            MotorToPosition(
                liftPivotMotor,
                round(PIVOT_TICKS_PER_DEGREE * angle).toInt(),
                PIVOT_SPEED
            )
        fun toPosition(position: Vector2d): AtomicCommand =
            toAngle(MecanumDrive.poseEstimate.vec() angleBetween position)
    }

    object LimitSwitch {
        @JvmField
        var SWITCH_NAME = "limitSwitch"

        private lateinit var limitSwitch: DigitalChannel

        fun initialize() {
            limitSwitch = opMode.hardwareMap.get(DigitalChannel::class.java, SWITCH_NAME)
        }

        val state: Boolean
            get() = limitSwitch.state
    }
}