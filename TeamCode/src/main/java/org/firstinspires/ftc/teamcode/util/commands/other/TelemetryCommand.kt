package org.firstinspires.ftc.teamcode.util.commands.other

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand

class TelemetryCommand(private val time: Double, private val message: String) : AtomicCommand() {
    override val _isDone: Boolean
        get() = timer.seconds() > time

    constructor(time: Double, caption: String, data: String) : this(time, "$caption: $data")

    override fun start() {
        timer.reset()
    }

    override fun execute() {
        Constants.opMode.telemetry.addLine(message)
    }

    val timer = ElapsedTime()
}