package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand

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

    var servoName = "detectionServo"
    var mb1220Name =  "mb1220"

    fun initialize() {
        servo = Constants.opMode.hardwareMap.get(Servo::class.java, servoName)
        mb1220 = Constants.opMode.hardwareMap.get(AnalogInput::class.java, mb1220Name)
    }

    class DetectCommand : AtomicCommand() {
        override val _isDone: Boolean
            get() = position != Position.UNKNOWN

        private var triedRight = false
        private val timer = ElapsedTime()

        override fun start() {
            position = Position.UNKNOWN
            timer.reset()
            servo.position = 0.09
        }

        override fun execute() {
            if (timer.seconds() > 1.0 && position == Position.UNKNOWN) {
                timer.reset()
                if (!triedRight) {
                    triedRight = true
                    if (mb1220.voltage < 0.09)
                        position = Position.RIGHT
                    else {
                        servo.position = 0.24
                    }
                }
                else {
                    position = if (mb1220.voltage < 0.12)
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