package org.firstinspires.ftc.teamcode.util.commands.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import kotlin.math.sign

@Suppress("MemberVisibilityCanBePrivate")
open class MotorToPosition(protected val motor: DcMotor, protected val position: Int,
                           protected var speed: Double, protected val minError: Int = 15,
                           protected val kP: Double = 0.005) : AtomicCommand() {
    override val _isDone: Boolean
        get() = error < minError

    protected var error: Int = 0
    protected var direction: Int = 0

    override fun start() {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        error = position - motor.currentPosition
        direction = sign(error.toDouble()).toInt()
    }

    override fun execute() {
        error = position - motor.currentPosition
        val power = kP * error * speed * direction
        motor.power = power
    }
}