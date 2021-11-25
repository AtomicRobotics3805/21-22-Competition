package org.firstinspires.ftc.teamcode.subsystems.localization

/*
import com.acmerobotics.roadrunner.geometry.Pose2d as RoadRunnerPose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.robotcore.util.ElapsedTime
import edu.wpi.first.wpilibj.estimator.MecanumDrivePoseEstimator
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics
import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.numbers.N1
import edu.wpi.first.wpiutil.math.numbers.N3

class PoseEstimator : Localizer {
    override var poseEstimate: RoadRunnerPose2d
        get() = TODO("Not yet implemented")
        set(value) {}
    override val poseVelocity: RoadRunnerPose2d?
        get() = TODO("Not yet implemented")

    val timer = ElapsedTime()


    lateinit var mecanumDrivePoseEstimator: MecanumDrivePoseEstimator

    fun initialize(gyroAngle: Rotation2d,
                   initialPoseMeters: Pose2d,
                   kinematics: MecanumDriveKinematics,
                   stateStdDevs: Matrix<N3, N1>,
                   localMeasurementStdDevs: Matrix<N1, N1>,
                   visionMeasurementStdDevs: Matrix<N3, N1>
    ) {
        mecanumDrivePoseEstimator = MecanumDrivePoseEstimator(gyroAngle, initialPoseMeters,
            kinematics, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs)
        timer.reset()
    }

    override fun update() {
        //mecanumDrivePoseEstimator.updateWithTime(timer.seconds(), )
    }

}
 */