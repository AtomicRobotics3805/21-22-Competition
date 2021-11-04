package org.firstinspires.ftc.teamcode.testing

import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand

object MB1220 {
    private lateinit var mb1220: AnalogInput
    private var voltage: Double = 0.0
    private var freight = false
    private const val cmThreshold = 0 //Modify after testing.
    private const val deviceName  = "MB1220"

    class Read: AtomicCommand() {
        override fun start(){
            mb1220 = opMode.hardwareMap.get(AnalogInput::class.java, deviceName)
        }
        override fun execute() {

            voltage = (mb1220.voltage) / 0.00468 //Scale factor for 5v.

            if (voltage < cmThreshold) {
                freight = true
            }
        }
    }
}