package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.TimedCustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.MotorToPosition
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Carousel : Subsystem {
    @JvmField
    var CAROUSEL_SLOW_POSITION = 13.0 * Math.PI
    @JvmField
    var CAROUSEL_FAST_POSITION = 18.0 * Math.PI
    @JvmField
    var COUNTS_PER_MOTOR_REV = 537.6
    @JvmField
    var WHEEL_DIAMETER = 4.0
    val COUNTS_PER_INCH: Double
            get() = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER * Math.PI)
    val CAROUSEL_SLOW_ROTATION_COUNTS: Int
        get() = (CAROUSEL_SLOW_POSITION * COUNTS_PER_INCH).toInt()
    val CAROUSEL_FAST_ROTATION_COUNTS: Int
        get() = (CAROUSEL_FAST_POSITION * COUNTS_PER_INCH).toInt()

    @JvmField
    var CAROUSEL_NAME = "carousel"
    @JvmField
    var CAROUSEL_DIRECTION = DcMotorSimple.Direction.REVERSE
    @JvmField
    var CAROUSEL_SLOW_SPEED = 0.6
    @JvmField
    var CAROUSEL_FAST_SPEED = 1.0

    var on = false

    val switch: AtomicCommand
        get() = if (on) stop else start
    val start: AtomicCommand
        get() = powerCarousel((if (CAROUSEL_DIRECTION == DcMotorSimple.Direction.FORWARD)
            1 else -1) * CAROUSEL_SLOW_SPEED)
    val stop: AtomicCommand
        get() = powerCarousel(0.0)
    val fullRotation: AtomicCommand
        get() = FullRotation(1)//TimedCustomCommand(_start = {motor.power = CAROUSEL_SLOW_SPEED}, _done = {motor.power = 0.0}, time = 2.3)
    val fullRotationReverse: AtomicCommand
        get() = FullRotation(-1)//TimedCustomCommand(_start = {motor.power = -CAROUSEL_SLOW_SPEED}, _done = {motor.power = 0.0}, time = 2.3)

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

    class FullRotation(direction: Int) : MotorToPosition(motor, CAROUSEL_FAST_ROTATION_COUNTS * direction, CAROUSEL_SLOW_SPEED) {
        override fun execute() {
            super.execute()
            if (error < CAROUSEL_FAST_ROTATION_COUNTS - CAROUSEL_SLOW_ROTATION_COUNTS)
                speed = CAROUSEL_FAST_SPEED
        }
    }
}