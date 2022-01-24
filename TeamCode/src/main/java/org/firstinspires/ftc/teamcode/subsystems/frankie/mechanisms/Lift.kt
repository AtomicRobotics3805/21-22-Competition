package org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetectionMB1220
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.delays.WaitUntil
import org.firstinspires.ftc.teamcode.util.commands.other.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.sequential
import org.firstinspires.ftc.teamcode.util.commands.subsystems.MotorToPosition
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.util.toDegrees
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
    }

    @Config
    object Extender : Subsystem {
        @JvmField
        var EXTENDER_NAME = "armExtend"

        @JvmField
        var FULL_EXTENSION_DISTANCE = 30.5

        @JvmField
        var OPEN_LATCH_DISTANCE = 25.0

        @JvmField
        var COLLECT_DISTANCE = 4.3

        @JvmField
        var EXTENDER_SPEED = 1.0

        @JvmField
        var EXTENDER_DIRECTION = DcMotorSimple.Direction.REVERSE

        @JvmField
        var startingDistance = 3.5

        private const val PULLEY_DIAMETER = 1.25
        private const val PULLEY_CIRCUMFERENCE: Double = PULLEY_DIAMETER * PI
        private const val EXTENDER_TICKS_PER_REV: Double = 28 * 3.7
        val EXTENDER_TICKS_PER_INCH: Int =
            round(EXTENDER_TICKS_PER_REV / PULLEY_CIRCUMFERENCE).toInt()

        lateinit var extensionMotor: DcMotorEx
        private var fullExtended = false

        fun initialize() {
            extensionMotor = opMode.hardwareMap.get(DcMotorEx::class.java, EXTENDER_NAME)
            extensionMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            extensionMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            extensionMotor.direction = EXTENDER_DIRECTION
        }

        val fullExtend: AtomicCommand
            get() = sequential {
                +ToPosition(FULL_EXTENSION_DISTANCE, _fullExtended = true, time = 3.0, kP = 0.01)
                +idle
            }
        val fullExtendStopEarly: AtomicCommand
            get() = sequential {
                +ToPosition(FULL_EXTENSION_DISTANCE, true)
                +idle
            }
        val extendOpenLatchDelay: AtomicCommand
            get() = WaitUntil { extensionMotor.currentPosition >= OPEN_LATCH_DISTANCE * EXTENDER_TICKS_PER_INCH }
        val retractAtStart: AtomicCommand
            get() = sequential {
                +ToPosition(0.0, minError = 3, kP = 0.08, time = 1.5)
                +idle
            }
        val retract: AtomicCommand
            get() = sequential {
                +ToPosition(0.0, _fullExtended = false, overrideSpeed = 1.0)
                +idle
            }
        val collect: AtomicCommand
            get() = sequential {
                +ToPosition(COLLECT_DISTANCE)
                +idle
            }
        val switch: AtomicCommand
            get() = if (fullExtended) retract else fullExtend
        val manualUp: AtomicCommand
            get() = powerExtender(EXTENDER_SPEED)
        val manualDown: AtomicCommand
            get() = powerExtender(-EXTENDER_SPEED)
        val idle: AtomicCommand
            get() = CustomCommand(_start = {
                extensionMotor.power = EXTENDER_SPEED
                extensionMotor.targetPosition = extensionMotor.currentPosition
                extensionMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            })

        fun powerExtender(power: Double) = CustomCommand(_start = {
            extensionMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            extensionMotor.power = power
        })

        class ToPosition(
            val distance: Double,
            val stopEarly: Boolean = false,
            val _fullExtended: Boolean? = null,
            minError: Int = 15,
            kP: Double = 0.005,
            overrideSpeed: Double = EXTENDER_SPEED,
            time: Double = 5.0
        ) : MotorToPosition(
            extensionMotor, round(
                EXTENDER_TICKS_PER_INCH * (distance - startingDistance)
            ).toInt(), overrideSpeed, minError, kP, time = time
        ) {
            override val _isDone: Boolean
                get() = if (!stopEarly) super._isDone else (distance * EXTENDER_TICKS_PER_REV) / abs(
                    error
                ) < 3

            override fun start() {
                if (_fullExtended != null) fullExtended = _fullExtended
                super.start()
            }
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
    object Swivel : Subsystem {
        @JvmField
        var SWIVEL_NAME = "armSwivel"

        @JvmField
        var SWIVEL_SPEED = 1.0

        @JvmField
        var SWIVEL_DIRECTION = DcMotorSimple.Direction.FORWARD

        @JvmField
        var START_DEGREES = 20

        @JvmField
        var UP_SLIGHTLY_DEGREES = 75

        @JvmField
        var LOW_DEGREES = 86

        @JvmField
        var MIDDLE_DEGREES = 93

        @JvmField
        var HIGH_DEGREES = 98

        @JvmField
        var ACCEPTABLE_PIVOT_ANGLE = 5.0

        @JvmField
        var ACCEPTABLE_HEIGHT = 50

        @JvmField
        var COLLECT_HEIGHT = 20


        private const val SWIVEL_GEAR_RATIO = 6.0
        private const val SWIVEL_TICKS_PER_REV: Double = 28 * 57.6
        val SWIVEL_TICKS_PER_DEGREE: Double =
            SWIVEL_TICKS_PER_REV * SWIVEL_GEAR_RATIO / 360.0
        private val LOW_POSITION: Int
            get() = round(SWIVEL_TICKS_PER_DEGREE * (LOW_DEGREES - START_DEGREES)).toInt()
        private val MIDDLE_POSITION: Int
            get() = round(SWIVEL_TICKS_PER_DEGREE * (MIDDLE_DEGREES - START_DEGREES)).toInt()
        private val HIGH_POSITION: Int
            get() = round(SWIVEL_TICKS_PER_DEGREE * (HIGH_DEGREES - START_DEGREES)).toInt()
        private val ACCEPTABLE_HEIGHT_TICKS: Int
            get() = round(SWIVEL_TICKS_PER_DEGREE * (ACCEPTABLE_HEIGHT - START_DEGREES)).toInt()
        private val COLLECT_POSITION: Int
            get() = round(SWIVEL_TICKS_PER_DEGREE * (COLLECT_HEIGHT - START_DEGREES)).toInt()
        private val UP_SLIGHTLY_POSITION: Int
            get() = round(SWIVEL_TICKS_PER_DEGREE * (UP_SLIGHTLY_DEGREES - START_DEGREES)).toInt()

        lateinit var swivelMotor: DcMotor

        fun initialize() {
            swivelMotor = opMode.hardwareMap.get(DcMotor::class.java, SWIVEL_NAME)
            swivelMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            swivelMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            swivelMotor.direction = SWIVEL_DIRECTION
        }

        val toStart: AtomicCommand
            get() = sequential {
                +MotorToPosition(swivelMotor, 0, SWIVEL_SPEED)
                +idle
            }
        val toLow: AtomicCommand
            get() = sequential {
                +MotorToPosition(swivelMotor, LOW_POSITION, SWIVEL_SPEED)
                +idle
            }
        val toMiddle: AtomicCommand
            get() = sequential {
                +MotorToPosition(swivelMotor, MIDDLE_POSITION, SWIVEL_SPEED)
                +idle
            }
        val toHigh: AtomicCommand
            get() = sequential {
                +MotorToPosition(swivelMotor, HIGH_POSITION, SWIVEL_SPEED)
                +idle
            }
        val upSlightly: AtomicCommand
            get() = sequential {
                +MotorToPosition(swivelMotor, UP_SLIGHTLY_POSITION, SWIVEL_SPEED)
                +idle
            }
        val toPreloadPosition: AtomicCommand
            get() = sequential {
                +ToPreloadPosition()
                +idle
            }
        val toPivotHeight: AtomicCommand
            get() = sequential {
                +MotorToPosition(swivelMotor, ACCEPTABLE_HEIGHT_TICKS, SWIVEL_SPEED)
                +idle
            }
        val manualUp: AtomicCommand
            get() = powerSwivel(SWIVEL_SPEED)
        val manualDown: AtomicCommand
            get() = powerSwivel(-SWIVEL_SPEED)
        val idle: AtomicCommand
            get() = CustomCommand(_start = {
                swivelMotor.power = SWIVEL_SPEED
                swivelMotor.targetPosition = swivelMotor.currentPosition
                swivelMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            })
        val toCollect: AtomicCommand
            get() = sequential {
                +MotorToPosition(swivelMotor, COLLECT_POSITION, SWIVEL_SPEED)
                +idle
            }
        val downPivotHeightDelay: AtomicCommand
            get() = WaitUntil { swivelMotor.currentPosition <= ACCEPTABLE_HEIGHT_TICKS }
        val upPivotHeightDelay: AtomicCommand
            get() = WaitUntil { swivelMotor.currentPosition >= ACCEPTABLE_HEIGHT_TICKS }

        fun powerSwivel(power: Double) = CustomCommand(_start = {
            swivelMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            swivelMotor.power = power
        })

        override fun periodic() {

        }

        class ToPreloadPosition :
            MotorToPosition(swivelMotor, HIGH_POSITION, SWIVEL_SPEED) {
            override fun start() {
                position = if (Constants.objectPosition == Constants.ObjectPosition.LEFT) LOW_POSITION
                else if (Constants.objectPosition == Constants.ObjectPosition.MIDDLE) MIDDLE_POSITION
                else HIGH_POSITION
                super.start()
            }
        }

        class ToCollectCareful :
            MotorToPosition(swivelMotor, ACCEPTABLE_HEIGHT_TICKS, SWIVEL_SPEED) {
            override fun execute() {
                position = if (abs(Pivot.angle) > ACCEPTABLE_PIVOT_ANGLE)
                    0
                else ACCEPTABLE_HEIGHT_TICKS
                super.execute()
            }
        }
    }

    @Config
    object Pivot {
        @JvmField
        var PIVOT_NAME = "armPivot"
        @JvmField
        var PIVOT_SPEED = 1.0
        @JvmField
        var PIVOT_DIRECTION = DcMotorSimple.Direction.FORWARD
        @JvmField
        var RELATIVE_POSITION = Vector2d(-8.0, 0.0)

        private const val PIVOT_GEAR_RATIO = 1.0
        private const val PIVOT_TICKS_PER_REV: Double = 28 * 19.2 * 3
        private const val PIVOT_TICKS_PER_DEGREE: Double =
            PIVOT_TICKS_PER_REV * PIVOT_GEAR_RATIO / 360.0

        lateinit var liftPivotMotor: DcMotorEx

        fun initialize() {
            liftPivotMotor = opMode.hardwareMap.get(DcMotorEx::class.java, PIVOT_NAME)
            liftPivotMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            liftPivotMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            liftPivotMotor.direction = PIVOT_DIRECTION
            liftPivotMotor.setVelocityPIDFCoefficients(14.0, 0.6, 1.0, 0.0)
            liftPivotMotor.setPositionPIDFCoefficients(23.0)
            liftPivotMotor.targetPositionTolerance = 1
        }

        val angle: Double
            get() = liftPivotMotor.currentPosition / PIVOT_TICKS_PER_DEGREE
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

        fun powerPivot(power: Double) = CustomCommand(_start = {
            liftPivotMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            liftPivotMotor.power = power
        })

        fun toAngle(angle: Double): AtomicCommand = ToAngle(angle)

        /*MotorToPosition(
            liftPivotMotor,
            round(PIVOT_TICKS_PER_DEGREE * angle).toInt(),
            PIVOT_SPEED
        )*/
        fun toPosition(position: Vector2d, robotPosition: Pose2d = MecanumDrive.poseEstimate): AtomicCommand =
            toAngle(
                numberToAngle(
                    ((
                            position angleBetween (robotPosition.vec() + RELATIVE_POSITION))
                            - MecanumDrive.poseEstimate.heading).toDegrees + 90.0
                )
            )

        fun numberToAngle(number: Double): Double {
            var angle = number
            while (angle > 180) angle -= 360
            while (angle <= -180) angle += 360
            return angle
        }

        class ToAngle(val angle: Double, val timeout: Double = 1.5): AtomicCommand() {

            override val _isDone: Boolean
                get() = !liftPivotMotor.isBusy || timer.seconds() >= timeout

            val timer = ElapsedTime()

            override fun start() {
                timer.reset()
                liftPivotMotor.targetPosition = round(PIVOT_TICKS_PER_DEGREE * angle).toInt()
                liftPivotMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                liftPivotMotor.power = PIVOT_SPEED
                if (angle == 0.0) {
                    liftPivotMotor.setVelocityPIDFCoefficients(10.0, 0.3, 0.0, 0.0)
                    liftPivotMotor.setPositionPIDFCoefficients(11.0)
                } else {
                    liftPivotMotor.setVelocityPIDFCoefficients(14.0, 0.6, 1.0, 0.0)
                    liftPivotMotor.setPositionPIDFCoefficients(25.0)
                }
            }
        }
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