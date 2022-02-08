package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem


@Suppress("unused")
@Config
object ContainerSensor : Subsystem {
    @JvmField
    var CONTAINER_SENSOR_NAME = "sensor_color"

    enum class ContainerState {
        EMPTY,
        FULL,
        UNKNOWN
    }

    val detect: AtomicCommand
        get() = ContainerDetectCommand()

    var containerState = ContainerState.UNKNOWN
    private lateinit var containerSensor: NormalizedColorSensor

    fun initialize() {
        containerSensor = Constants.opMode.hardwareMap.get(NormalizedColorSensor::class.java, CONTAINER_SENSOR_NAME)
    }

    class ContainerDetectCommand : AtomicCommand() {
        override val _isDone = false

        override fun execute() {
            containerState =
                if ((containerSensor as DistanceSensor).getDistance(DistanceUnit.CM) > 3.5) {
                    ContainerState.EMPTY
                }
                else ContainerState.FULL
        }

        override fun done(interrupted: Boolean) {
            containerState = ContainerState.EMPTY
        }
    }
}