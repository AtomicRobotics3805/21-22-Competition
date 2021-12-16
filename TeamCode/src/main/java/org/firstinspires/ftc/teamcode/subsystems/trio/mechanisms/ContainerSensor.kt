package org.firstinspires.ftc.teamcode.subsystems.trio.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem


@Suppress("unused")
@Config
object ContainerSensor : Subsystem {
    @JvmField
    var CONTAINER_SENSOR_NAME = "containerSensor"
    @JvmField
    var MAXIMUM_FULL_VOLTAGE = 0.1
    @JvmField
    var MAXIMUM_ERROR_VOLTAGE = 0.0001

    enum class ContainerState {
        EMPTY,
        FULL,
        UNKNOWN
    }

    val detect: AtomicCommand
        get() = ContainerDetectCommand()

    var containerState = ContainerState.UNKNOWN
    private lateinit var containerSensor: AnalogInput

    fun initialize() {
        containerSensor = Constants.opMode.hardwareMap.get(AnalogInput::class.java, CONTAINER_SENSOR_NAME)
    }

    class ContainerDetectCommand : AtomicCommand() {
        override val _isDone = false

        override fun execute() {
            val voltage = containerSensor.voltage
            containerState =
                when {
                    voltage < MAXIMUM_ERROR_VOLTAGE -> ContainerState.UNKNOWN
                    voltage < MAXIMUM_FULL_VOLTAGE -> ContainerState.FULL
                    else -> ContainerState.EMPTY
                }
        }

        override fun done(interrupted: Boolean) {
            containerState = ContainerState.EMPTY
        }
    }
}