package org.firstinspires.ftc.teamcode.testing

import android.app.Activity
import android.graphics.Color
import android.view.View
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import com.qualcomm.robotcore.hardware.LED
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object BucketIndicator {

    enum class Bucket {
        FULL,
        EMPTY,
        UNKNOWN
    }

    var status: Bucket = Bucket.UNKNOWN
    private lateinit var distanceSensor: DistanceSensor
    private lateinit var colorSensor: ColorSensor

    private lateinit var ledZero: LED
    private lateinit var ledOne: LED
    private lateinit var ledTwo: LED
    private lateinit var ledThree: LED

    var distanceSensorName = "color_distance_sensor"
    var colorSensorName = "color_distance_sensor"

    var ledZeroName = "led_0"
    var ledOneName = "led_1"
    var ledTwoName = "led_2"
    var ledThreeName = "led_3"

    private var distance = 0.0
    private val distanceMax = 3.3

    val hsvValues = floatArrayOf(0f, 0f, 0f)
    val scaleFactor = 255
    var relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());

        fun initialize() {

            distanceSensor = opMode.hardwareMap.get(DistanceSensor::class.java,distanceSensorName)
            colorSensor = opMode.hardwareMap.get(ColorSensor::class.java,colorSensorName)

            ledZero = opMode.hardwareMap.get(LED::class.java,ledZeroName)
            ledOne = opMode.hardwareMap.get(LED::class.java,ledOneName)
            ledTwo = opMode.hardwareMap.get(LED::class.java,ledTwoName)
            ledThree = opMode.hardwareMap.get(LED::class.java,ledThreeName)

            status = Bucket.UNKNOWN
        }
        class DetectCommmand : AtomicCommand(){
            override val _isDone: Boolean
                get() = status != Bucket.UNKNOWN

            override fun start() {
                status = Bucket.UNKNOWN
            }
            override fun execute() {
                distance = distanceSensor.getDistance(DistanceUnit.CM)
                Color.RGBToHSV((colorSensor.red() * scaleFactor) as Int,
                        (colorSensor.green() * scaleFactor) as Int,
                        (colorSensor.blue() * scaleFactor) as Int,
                        hsvValues)

                if (distance < 3.3) {
                    status = if (75 < hsvValues[0] && hsvValues[0] < 79 || 93 < hsvValues[0] && hsvValues[0] < 97 || 163 < hsvValues[0] && hsvValues[0] < 168) {
                        Bucket.FULL
                    }
                    else{
                        Bucket.EMPTY
                    }
                }
            }
        }
}

