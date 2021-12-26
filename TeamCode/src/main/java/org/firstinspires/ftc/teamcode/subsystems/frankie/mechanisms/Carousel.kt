package org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.CustomGamepad
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.MotorToPosition
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import kotlin.math.abs

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Carousel : Subsystem {
    @JvmField
    var CAROUSEL_NAME = "carousel"
    @JvmField
    var CAROUSEL_DIRECTION = DcMotorSimple.Direction.REVERSE

    // used when doing a full automatic rotation
    @JvmField
    var CAROUSEL_AUTOMATIC_SLOW_SPEED = 0.4
    // used when doing a full automatic rotation
    @JvmField
    var CAROUSEL_AUTOMATIC_FAST_SPEED = 1.0
    // used when pressing a button to rotate the carousel manually
    @JvmField
    var CAROUSEL_MANUAL_SPEED = 0.55
    // multiplied by the value of a trigger when rotating the carousel manually
    @JvmField
    var CAROUSEL_TRIGGER_SPEED = 0.7
    @JvmField

    var CAROUSEL_SLOW_POSITION: Double = 13.0 * Math.PI
    @JvmField
    var CAROUSEL_FAST_POSITION: Double = 24.0 * Math.PI
    @JvmField
    var COUNTS_PER_MOTOR_REV: Double = 28 * 3.7
    @JvmField
    var WHEEL_DIAMETER = 4.0

    val COUNTS_PER_INCH: Double
            get() = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER * Math.PI)
    val CAROUSEL_SLOW_ROTATION_COUNTS: Int
        get() = (CAROUSEL_SLOW_POSITION * COUNTS_PER_INCH).toInt()
    val CAROUSEL_FAST_ROTATION_COUNTS: Int
        get() = (CAROUSEL_FAST_POSITION * COUNTS_PER_INCH).toInt()

    var on = false

    val switch: AtomicCommand
        get() = if (on) stop else start
    val start: AtomicCommand
        get() = powerCarousel(if (Constants.color == Constants.Color.BLUE)
            CAROUSEL_MANUAL_SPEED else -CAROUSEL_MANUAL_SPEED)
    val stop: AtomicCommand
        get() = powerCarousel(0.0)
    val fullRotation: AtomicCommand
        get() = if (Constants.color == Constants.Color.BLUE)
            FullRotation(1) else FullRotation(-1)

    private lateinit var carouselMotor: DcMotorEx

    fun initialize() {
        carouselMotor = opMode.hardwareMap.get(DcMotorEx::class.java, CAROUSEL_NAME)
        carouselMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        carouselMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        carouselMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    fun powerCarouselTrigger(trigger: CustomGamepad.Trigger) = CustomCommand(_start = {
        carouselMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }, _execute = {
        carouselMotor.power = trigger.amount.toDouble() * CAROUSEL_TRIGGER_SPEED
    })

    fun powerCarousel(power: Double) = CustomCommand(_start = {
        carouselMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        carouselMotor.power = power
        on = power != 0.0
    })

    class FullRotation(direction: Int) : MotorToPosition(
        carouselMotor,
        CAROUSEL_FAST_ROTATION_COUNTS * direction + carouselMotor.currentPosition,
        CAROUSEL_AUTOMATIC_SLOW_SPEED
    ) {
        override fun execute() {
            super.execute()
            if (abs(error) < CAROUSEL_FAST_ROTATION_COUNTS - CAROUSEL_SLOW_ROTATION_COUNTS)
                speed = CAROUSEL_AUTOMATIC_FAST_SPEED
        }
    }
}