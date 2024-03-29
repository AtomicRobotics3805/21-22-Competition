package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.other.CustomCommand
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
    val intakeIfEmpty: AtomicCommand
        get() = CustomCommand(_start = {
            if (ContainerSensor.containerState == ContainerSensor.ContainerState.EMPTY) {
                intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                intakeMotor.power = INTAKE_SPEED
            }
        })
    val intakeOne: AtomicCommand
        get() = IntakeOne()

    var on = false
    private lateinit var intakeMotor: DcMotorEx

    fun initialize() {
        intakeMotor = opMode.hardwareMap.get(DcMotorEx::class.java, INTAKE_NAME)
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    fun powerIntake(power: Double) = CustomCommand(_start = {
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor.power = power
        on = power != 0.0
    })

    class IntakeOne : AtomicCommand() {
        override val _isDone
            get() = !intakeMotor.isBusy

        override fun start() {
            intakeMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            intakeMotor.targetPosition = ROTATION_COUNTS
            intakeMotor.power = INTAKE_SPEED
        }
    }
}