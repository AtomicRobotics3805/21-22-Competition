package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Arm : Subsystem {
    @JvmField
    var ARM_NAME = "arm"
    @JvmField
    var ARM_SPEED = 1.0
    @JvmField
    var ARM_POSITION_HIGH = 22.0 // in
    @JvmField
    var ARM_POSITION_MEDIUM = 0.0 // in
    @JvmField
    var ARM_POSITION_LOW = 0.0 // in

    private const val PULLEY_WIDTH = 2.0 // in
    private const val COUNTS_PER_REV = 537.6
    // higher value makes driven gear slower
    private const val DRIVE_GEAR_REDUCTION = 1.0
    private const val COUNTS_PER_INCH = COUNTS_PER_REV * DRIVE_GEAR_REDUCTION / (PULLEY_WIDTH * Math.PI)

    val toLow: AtomicCommand
        get() = moveArmToPosition((ARM_POSITION_LOW * COUNTS_PER_INCH).toInt())
    val toMedium: AtomicCommand
        get() = moveArmToPosition((ARM_POSITION_MEDIUM * COUNTS_PER_INCH).toInt())
    val toHigh: AtomicCommand
        get() = moveArmToPosition((ARM_POSITION_HIGH * COUNTS_PER_INCH).toInt())
    val toStart: AtomicCommand
        get() = moveArmToPosition(0)
    val start: AtomicCommand
        get() = powerArm(ARM_SPEED)
    val reverse: AtomicCommand
        get() = powerArm(-ARM_SPEED)
    val stop: AtomicCommand
        get() = halt()

    private lateinit var motor: DcMotorEx

    fun initialize() {
        motor = opMode.hardwareMap.get(DcMotorEx::class.java, ARM_NAME)
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun halt() = CustomCommand(_start = {
        //motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        //motor.targetPosition = motor.currentPosition
        motor.power = 0.0
    })

    fun powerArm(speed: Double) = CustomCommand(_start = {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.power = speed
    })

    fun moveArmToPosition(position: Int) = CustomCommand(
            getDone = { !motor.isBusy },
            _start = {
                motor.mode = DcMotor.RunMode.RUN_TO_POSITION
                motor.targetPosition = position
                motor.power = ARM_SPEED
            }
    )
}