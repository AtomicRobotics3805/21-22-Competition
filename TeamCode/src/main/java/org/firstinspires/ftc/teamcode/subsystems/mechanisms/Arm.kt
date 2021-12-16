package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetectionMB1220
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Arm : Subsystem {
    @JvmField
    var ARM_NAME = "lift"
    @JvmField
    var ARM_SPEED = 1.0
    @JvmField
    var ARM_POSITION_HIGH = 22.0 // in
    @JvmField
    var ARM_POSITION_MEDIUM = 14.4 // in
    @JvmField
    var ARM_POSITION_LOW = 9.0 // in

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
        get() = moveArmToPosition((0.1 * COUNTS_PER_INCH).toInt())
    val start: AtomicCommand
        get() = powerArm(ARM_SPEED)
    val reverse: AtomicCommand
        get() = powerArm(-ARM_SPEED)
    val stop: AtomicCommand
        get() = halt()

    private lateinit var armMotor: DcMotorEx

    fun initialize() {
        armMotor = opMode.hardwareMap.get(DcMotorEx::class.java, ARM_NAME)
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun halt() = CustomCommand(_start = {
        //armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        //armMotor.targetPosition = armMotor.currentPosition
        armMotor.power = 0.0
    })

    fun powerArm(speed: Double) = CustomCommand(_start = {
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.power = speed
    })

    fun moveArmToPosition(position: Int) = CustomCommand(
            getDone = { !armMotor.isBusy },
            _start = {
                armMotor.targetPosition = position
                armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                armMotor.power = ARM_SPEED
            }
    )

    fun moveArmAutonomous() = CustomCommand(
        getDone = { !armMotor.isBusy },
        _start = {
            if (ObjectDetectionMB1220.position == ObjectDetectionMB1220.Position.LEFT)
                armMotor.targetPosition = (ARM_POSITION_LOW * COUNTS_PER_INCH).toInt()
            else if (ObjectDetectionMB1220.position == ObjectDetectionMB1220.Position.MIDDLE)
                    armMotor.targetPosition = (ARM_POSITION_MEDIUM * COUNTS_PER_INCH).toInt()
            else armMotor.targetPosition = (ARM_POSITION_HIGH * COUNTS_PER_INCH).toInt()
            armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            armMotor.power = ARM_SPEED
        }
    )
}