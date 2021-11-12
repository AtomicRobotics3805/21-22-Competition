package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Intake : Subsystem {
    @JvmField
    var INTAKE_NAME = "intake"
    @JvmField
    var INTAKE_SPEED = 1.0

    private const val COUNTS_PER_MOTOR_REV = 537.6
    // higher value makes driven gear slower
    private const val DRIVE_GEAR_REDUCTION = 1.0
    private const val ROTATION_DEGREES = 500
    private const val ROTATION_COUNTS = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION *
            (ROTATION_DEGREES / 360.0)).toInt()

    val start: AtomicCommand
        get() = powerIntake(INTAKE_SPEED)
    val stop: AtomicCommand
        get() = powerIntake(0.0)
    val switch: AtomicCommand
        get() = if (on) stop else start
    val intakeOne: AtomicCommand
        get() = IntakeOne()

    var on = false
    private lateinit var motor: DcMotorEx

    fun initialize() {
        motor = opMode.hardwareMap.get(DcMotorEx::class.java, INTAKE_NAME)
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.direction = DcMotorSimple.Direction.REVERSE
    }

    fun powerIntake(power: Double) = CustomCommand(_start = {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.power = power
        on = power != 0.0
    })

    class IntakeOne : AtomicCommand() {
        override val _isDone
            get() = !motor.isBusy

        override fun start() {
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.targetPosition = ROTATION_COUNTS
            motor.power = INTAKE_SPEED
        }
    }
}