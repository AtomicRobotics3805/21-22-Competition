package org.firstinspires.ftc.teamcode.util.commands.other

import org.firstinspires.ftc.teamcode.Constants

class TelemetryMessage(time: Double, message: String) :
    TimedCustomCommand(time, _start = { Constants.opMode.telemetry.addLine(message)})