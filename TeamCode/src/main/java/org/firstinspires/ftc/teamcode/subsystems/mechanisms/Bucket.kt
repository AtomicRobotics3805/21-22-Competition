package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.TimedCustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import kotlin.math.abs

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Bucket : Subsystem {
    @JvmField
    var BUCKET_NAME = "bucket"
    @JvmField
    var DROP_POSITION = 0.0
    @JvmField
    var UP_POSITION = 0.0

    val drop: AtomicCommand
        get() = moveServo(DROP_POSITION)
    val up: AtomicCommand
        get() = moveServo(UP_POSITION)

    private lateinit var servo: Servo

    fun initialize() {
        servo = opMode.hardwareMap.get(Servo::class.java, BUCKET_NAME)
    }

    fun moveServo(position: Double) =
            TimedCustomCommand(time = abs(position - servo.position),
                    _start = {
                        servo.position = position
                    })
}