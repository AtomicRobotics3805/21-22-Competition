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
object BucketIndicator : Subsystem {

    enum class Bucket {
        FULL,
        EMPTY
    }

    var status: Bucket = Bucket.EMPTY
    private lateinit var distanceSensor: DistanceSensor
    private lateinit var colorSensor: ColorSensor

    private lateinit var ledZero: LED
    private lateinit var ledOne: LED
    private lateinit var ledTwo: LED
    private lateinit var ledThree: LED

    private var distance = 0.0
    private val distanceMax = 3.3

    val hsvValues = floatArrayOf(0f, 0f, 0f)
    val scaleFactor = 255
    var relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());

    //val status: AtomicCommand
    //    get() = readSensor()

    fun initialize() {

        distanceSensor = opMode.hardwareMap.get(DistanceSensor::class.java,"color_distance_sensor")
        colorSensor = opMode.hardwareMap.get(ColorSensor::class.java,"color_distance_sensor")

        ledZero = opMode.hardwareMap.get(LED::class.java,"led_0" )
        ledOne = opMode.hardwareMap.get(LED::class.java,"led_1" )
        ledTwo = opMode.hardwareMap.get(LED::class.java,"led_2" )
        ledThree = opMode.hardwareMap.get(LED::class.java,"led_3" )

    }
    fun readSensor(){
        distance = distanceSensor.getDistance(DistanceUnit.CM)
        Color.RGBToHSV((colorSensor.red() * scaleFactor) as Int,
                (colorSensor.green() * scaleFactor) as Int,
                (colorSensor.blue() * scaleFactor) as Int,
                hsvValues)
    }
    fun enable(){
        distance = distanceSensor.getDistance(DistanceUnit.CM)
        Color.RGBToHSV((colorSensor.red() * scaleFactor) as Int,
                (colorSensor.green() * scaleFactor) as Int,
                (colorSensor.blue() * scaleFactor) as Int,
                hsvValues)

        if (distance < distanceMax){
            if (74 < hsvValues[0] && hsvValues[0] < 79
                    || 93 < hsvValues[0] && hsvValues[0] < 97
                    || 163 < hsvValues[0] && hsvValues[0] < 168){
                ledZero.enableLight(false)
                ledOne.enableLight(true)
                ledTwo.enableLight(false)
                ledThree.enableLight(true)
            }
            else {
                ledZero.enableLight(true)
                ledOne.enableLight(false)
                ledTwo.enableLight(true)
                ledThree.enableLight(false)
            }
        }
    }
}

