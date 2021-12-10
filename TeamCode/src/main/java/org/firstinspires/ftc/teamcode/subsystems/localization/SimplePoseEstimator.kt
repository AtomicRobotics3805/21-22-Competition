package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

class SimplePoseEstimator : Localizer {
    override var poseEstimate: Pose2d
        get() = odometryPositions.last().second + visionAdjustment
        set(value) {
            manualAdjustment = value - poseEstimate
        }
    override val poseVelocity: Pose2d?
        get() = odometryLocalizer.poseVelocity

    private val odometryPositions: ArrayList<Pair<Double, Pose2d>> = arrayListOf(Pair(0.0, Pose2d()))
    private lateinit var odometryLocalizer: OdometryLocalizer
    private var visionAdjustment = Pose2d()
    private var manualAdjustment = Pose2d()

    private val timer = ElapsedTime()

    fun initialize() {
        odometryLocalizer = OdometryLocalizer()
        timer.reset()
    }

    override fun update() {
        odometryLocalizer.update()
        odometryPositions.add(Pair(timer.milliseconds(), odometryLocalizer.poseEstimate))
    }

    fun addVisionMeasurement(position: Pose2d, delayTimeMillis: Double) {
        var nearestTime: Double? = null
        var nearestPair: Pair<Double, Pose2d>? = null
        for (pair in odometryPositions) {
            val time: Double = abs(timer.milliseconds() - delayTimeMillis - pair.first)
            if (nearestTime == null || time < nearestTime) {
                nearestPair = pair
                nearestTime = time
            }
        }
        if (nearestPair != null)
            visionAdjustment = position - nearestPair.second
    }
}