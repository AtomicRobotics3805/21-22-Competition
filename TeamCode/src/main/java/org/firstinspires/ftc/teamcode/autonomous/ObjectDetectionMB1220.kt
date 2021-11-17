package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand

@Config
object ObjectDetectionMB1220 {
    enum class Position {
        UNKNOWN,
        LEFT,
        MIDDLE,
        RIGHT
    }

    var position: Position = Position.UNKNOWN

    private lateinit var servo: Servo
    private lateinit var mb1220: AnalogInput

    @JvmField
    var servoName = "detectionServo"
    @JvmField
    var mb1220Name =  "mb1220"

    @JvmField
    var rightServoPosition = 0.08
    @JvmField
    var rightVoltage = 0.087
    @JvmField
    var middleServoPosition = 0.255
    @JvmField
    var middleVoltage = 0.087

    fun initialize() {
        servo = Constants.opMode.hardwareMap.get(Servo::class.java, servoName)
        mb1220 = Constants.opMode.hardwareMap.get(AnalogInput::class.java, mb1220Name)
        position = Position.UNKNOWN
    }

    class DetectCommand : AtomicCommand() {
        override val _isDone: Boolean
            get() = position != Position.UNKNOWN

        private var triedRight = false
        private val timer = ElapsedTime()

        override fun start() {
            position = Position.UNKNOWN
            timer.reset()
            servo.position = rightServoPosition
        }

        override fun execute() {
            if (timer.seconds() > 1.0 && position == Position.UNKNOWN) {
                timer.reset()
                if (!triedRight) {
                    triedRight = true
                    if (mb1220.voltage < rightVoltage)
                        position = Position.RIGHT
                    else {
                        servo.position = middleServoPosition
                    }
                }
                else {
                    position = if (mb1220.voltage < middleVoltage)
                        Position.MIDDLE
                    else Position.LEFT
                }
                Constants.opMode.telemetry.addData("Voltage", mb1220.voltage)
                Constants.opMode.telemetry.addData("Position", position)
                Constants.opMode.telemetry.update()
            }
        }
    }
}