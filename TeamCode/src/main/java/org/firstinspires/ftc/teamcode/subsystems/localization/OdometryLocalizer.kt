package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer.Constants.PARALLEL_X
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer.Constants.PARALLEL_Y
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer.Constants.PERPENDICULAR_X
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer.Constants.PERPENDICULAR_Y
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
class OdometryLocalizer : TwoTrackingWheelLocalizer(listOf(
        Pose2d(PARALLEL_X, PARALLEL_Y, 0.0),
        Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90.0))
)) {
    val perpendicularEncoder: Encoder = Encoder(opMode.hardwareMap.get(DcMotorEx::class.java, "LF"))
    val parallelEncoder: Encoder = Encoder(opMode.hardwareMap.get(DcMotorEx::class.java, "RF"))

    init {
        if (Constants.PERPENDICULAR_REVERSED) perpendicularEncoder.direction = Encoder.Direction.REVERSE
        if (Constants.PARALLEL_REVERSED) parallelEncoder.direction = Encoder.Direction.REVERSE
    }

    override fun getHeading(): Double {
        return MecanumDrive.rawExternalHeading
    }

    override fun getHeadingVelocity(): Double? {
        return MecanumDrive.getExternalHeadingVelocity()
    }

    override fun getWheelPositions(): List<Double> {
        return listOf(
                encoderTicksToInches(perpendicularEncoder.currentPosition.toDouble()) * Constants.X_MULTIPLIER,
                encoderTicksToInches(parallelEncoder.currentPosition.toDouble()) * Constants.Y_MULTIPLIER
        )
    }

    override fun getWheelVelocities(): List<Double> {
        // If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return listOf(
                encoderTicksToInches(perpendicularEncoder.rawVelocity) * Constants.X_MULTIPLIER,
                encoderTicksToInches(parallelEncoder.rawVelocity) * Constants.Y_MULTIPLIER
        )
    }

    private fun encoderTicksToInches(ticks: Double): Double {
        return Constants.WHEEL_RADIUS * 2 * Math.PI * Constants.GEAR_RATIO * ticks / Constants.TICKS_PER_REV
    }

    object Constants {
        @JvmField
        var TICKS_PER_REV = 2400.0
        @JvmField
        var WHEEL_RADIUS = 17.5 * 0.03937 // in
        @JvmField
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        @JvmField
        var PARALLEL_X = 0.0 // in; forward offset of the parallel wheel
        @JvmField
        var PARALLEL_Y = 0.0 // in; left offset of the parallel wheel
        @JvmField
        var PERPENDICULAR_X = 0.0 // in; forward offset of the perpendicular wheel
        @JvmField
        var PERPENDICULAR_Y = 0.0 // in; left offset of the perpendicular wheel
        @JvmField
        var PARALLEL_REVERSED = true
        @JvmField
        var PERPENDICULAR_REVERSED = true
        @JvmField
        var X_MULTIPLIER = 1.0
        @JvmField
        var Y_MULTIPLIER = 1.0
    }
}