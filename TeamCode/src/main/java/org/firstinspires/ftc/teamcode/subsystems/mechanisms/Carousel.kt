package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.other.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.other.TimedCustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Carousel : Subsystem {
    const val CAROUSEL_CIRCUMFERENCE = 15.0 * Math.PI
    const val COUNTS_PER_MOTOR_REV = 537.6
    const val WHEEL_DIAMETER = 4.0
    const val COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER * Math.PI)
    const val CAROUSEL_ROTATION_COUNTS = (CAROUSEL_CIRCUMFERENCE * (400.0/360.0) * COUNTS_PER_INCH).toInt()

    @JvmField
    var CAROUSEL_NAME = "carousel"
    @JvmField
    var CAROUSEL_DIRECTION = DcMotorSimple.Direction.REVERSE
    @JvmField
    var CAROUSEL_SPEED = 0.08
    @JvmField
    var CAROUSEL_BACKWARDS_SPEED = -0.08

    var on = false

    val switch: AtomicCommand
        get() = if (on) stop else start
    val start: AtomicCommand
        get() = powerCarousel((if (CAROUSEL_DIRECTION == DcMotorSimple.Direction.FORWARD)
            1 else -1) * CAROUSEL_SPEED)
    val stop: AtomicCommand
        get() = powerCarousel(0.0)
    val fullRotation: AtomicCommand
        get() = TimedCustomCommand(_start = {motor.power = CAROUSEL_SPEED}, _done = {motor.power = 0.0}, time = 2.3)
    val fullRotationReverse: AtomicCommand
        get() = TimedCustomCommand(_start = {motor.power = CAROUSEL_BACKWARDS_SPEED}, _done = {motor.power = 0.0}, time = 2.3)

    private lateinit var motor: DcMotorEx

    fun initialize() {
        motor = opMode.hardwareMap.get(DcMotorEx::class.java, CAROUSEL_NAME)
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.direction = DcMotorSimple.Direction.REVERSE
    }

    fun powerCarousel(power: Double) = CustomCommand(_start = {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.power = power
        on = power != 0.0
    })

    class FullRotation : AtomicCommand() {
        override val _isDone
            get() = !motor.isBusy

        override fun start() {
            motor.targetPosition = CAROUSEL_ROTATION_COUNTS
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.power = CAROUSEL_SPEED
        }
    }
}