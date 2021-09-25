package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Carousel : Subsystem {
    @JvmField
    var CAROUSEL_NAME = "carousel"
    @JvmField
    var CAROUSEL_DIRECTION = DcMotorSimple.Direction.REVERSE
    @JvmField
    var CAROUSEL_SPEED = 0.5

    var on = false

    val switch: AtomicCommand
        get() = if (on) start else stop
    val start: AtomicCommand
        get() = powerCarousel((if (CAROUSEL_DIRECTION == DcMotorSimple.Direction.FORWARD)
            1 else -1) * CAROUSEL_SPEED)
    val stop: AtomicCommand
        get() = powerCarousel(0.0)

    private lateinit var motor: DcMotorEx

    fun initialize() {
        motor = opMode.hardwareMap.get(DcMotorEx::class.java, CAROUSEL_NAME)
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun powerCarousel(power: Double) = CustomCommand(_start = {
        motor.power = power
        on = power != 0.0
    })
}