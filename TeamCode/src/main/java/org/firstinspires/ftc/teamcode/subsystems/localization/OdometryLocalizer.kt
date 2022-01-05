package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.roadrunner.Encoder

/*
* Sample tracking wheel localizer implementation assuming the standard configuration:
*
*    /--------------\
*    |     ____     |
*    |     ----     |
*    | ||        || |
*    | ||        || |
*    |              |
*    |              |
*    \--------------/
*
*/

@Config
class OdometryLocalizer(PARALLEL_X: Double, PARALLEL_Y: Double,
                        PERPENDICULAR_X: Double, PERPENDICULAR_Y: Double) :
    TwoTrackingWheelLocalizer(listOf(
        Pose2d(PARALLEL_X, PARALLEL_Y, 0.0),
        Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90.0))
)) {
    @JvmField
    var TICKS_PER_REV = 2400
    @JvmField
    var WHEEL_RADIUS = 1.5 // in
    @JvmField
    var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
    @JvmField
    var PARALLEL_REVERSED = true
    @JvmField
    var PERPENDICULAR_REVERSED = true
    @JvmField
    var X_MULTIPLIER = 1.028
    @JvmField
    var Y_MULTIPLIER = 1.0
    
    val perpendicularEncoder: Encoder = Encoder(opMode.hardwareMap.get(DcMotorEx::class.java, "LB"))
    val parallelEncoder: Encoder = Encoder(opMode.hardwareMap.get(DcMotorEx::class.java, "RB"))

    init {
        if (PERPENDICULAR_REVERSED) perpendicularEncoder.direction = Encoder.Direction.REVERSE
        if (PARALLEL_REVERSED) parallelEncoder.direction = Encoder.Direction.REVERSE
    }

    override fun getHeading(): Double {
        return MecanumDrive.rawExternalHeading
    }

    override fun getHeadingVelocity(): Double? {
        return MecanumDrive.getExternalHeadingVelocity()
    }

    override fun getWheelPositions(): List<Double> {
        return listOf(
                encoderTicksToInches(parallelEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.currentPosition.toDouble()) * Y_MULTIPLIER
        )
    }

    override fun getWheelVelocities(): List<Double> {
        // If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return listOf(
                encoderTicksToInches(perpendicularEncoder.rawVelocity) * X_MULTIPLIER,
                encoderTicksToInches(parallelEncoder.rawVelocity) * Y_MULTIPLIER
        )
    }

    private fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }
}