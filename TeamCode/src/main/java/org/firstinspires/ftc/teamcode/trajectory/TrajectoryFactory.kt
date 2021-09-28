package org.firstinspires.ftc.teamcode.trajectory

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.Constants.startPose
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.toRadians

@Suppress("unused", "MemberVisibilityCanBePrivate")
@Config
object TrajectoryFactory {

    val startToCarousel = MecanumDrive.trajectoryBuilder(startPose, startPose.heading)
            .splineToConstantHeading(Vector2d(-53.5, 62.0), 175.0.toRadians)
            .build()

    val startToPark = MecanumDrive.trajectoryBuilder(startPose, true)
            .splineToConstantHeading(Vector2d(40.0, 52.0), 0.0.toRadians)
            .build()

    val carouselToHub = MecanumDrive.trajectoryBuilder(startToCarousel.trajectory.end(), true)
            .splineToSplineHeading(Pose2d(-12.0, 42.0, 270.0.toRadians), 320.0.toRadians)
            .build()

    val hubToPark = MecanumDrive.trajectoryBuilder(carouselToHub.trajectory.end(), carouselToHub.trajectory.end().heading + 100.0.toRadians)
            .splineTo(Vector2d(40.0, 52.0), 10.0.toRadians)
            .build()

    val carouselToPark = MecanumDrive.trajectoryBuilder(startToCarousel.trajectory.end(), startToCarousel.trajectory.end().heading)
            .back(95.0)
            .build()
}