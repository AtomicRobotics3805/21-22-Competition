package org.firstinspires.ftc.teamcode.util.commands.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.round
import kotlin.math.sign

@Suppress("MemberVisibilityCanBePrivate")
open class MotorToPosition(protected val motor: DcMotor, protected var position: Int,
                           protected var speed: Double, protected val minError: Int = 15,
                           protected val kP: Double = 0.005) : AtomicCommand() {
    override val _isDone: Boolean
        get() = abs(error) < minError

    protected var error: Int = 0
    protected var direction: Double = 0.0

    override fun start() {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    override fun execute() {
        error = position - motor.currentPosition
        direction = sign(error.toDouble())
        val power = kP * abs(error) * speed * direction
        motor.power = Range.clip(power, -min(speed, 1.0), min(speed, 1.0))
        Constants.opMode.telemetry.addData("Error", error)
    }

    override fun done(interrupted: Boolean) {
        motor.power = 0.0
    }
}