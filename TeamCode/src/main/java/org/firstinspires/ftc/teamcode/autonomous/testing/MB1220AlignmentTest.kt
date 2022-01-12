package org.firstinspires.ftc.teamcode.autonomous.testing

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetectionMB1220

@TeleOp
@Disabled
class MB1220AlignmentTest : LinearOpMode() {

    private lateinit var servo: Servo
    private lateinit var mb1220: AnalogInput
    private val servoTimer = ElapsedTime()
    private val voltageTimer = ElapsedTime()
    private var servoPosition = 0.0
    private var minVoltage = 0.07

    override fun runOpMode() {
        servo = hardwareMap.get(Servo::class.java, ObjectDetectionMB1220.servoName)
        mb1220 = hardwareMap.get(AnalogInput::class.java, ObjectDetectionMB1220.mb1220Name)
        telemetry.addLine("Ready")
        telemetry.update()
        waitForStart()
        servoTimer.reset()
        voltageTimer.reset()
        while (opModeIsActive()) {
            servoPosition += gamepad1.left_stick_x * servoTimer.seconds() * 0.1
            servoPosition += gamepad1.left_stick_x * servoTimer.seconds() * 0.01
            servoTimer.reset()
            minVoltage += gamepad1.right_stick_x * voltageTimer.seconds() * 0.03
            minVoltage += gamepad1.right_stick_x * voltageTimer.seconds() * 0.003
            voltageTimer.reset()
            servo.position = servoPosition
            val voltage = mb1220.voltage
            telemetry.addData("Sees Object?", voltage > minVoltage)
            telemetry.addData("Servo Position", servoPosition)
            telemetry.addData("Voltage", voltage)
            telemetry.addData("Minimum Voltage", minVoltage)
            telemetry.update()
        }
    }


}