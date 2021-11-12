package org.firstinspires.ftc.teamcode.trajectory

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.trajectories.ParallelTrajectory

@Suppress("unused", "MemberVisibilityCanBePrivate")
@Config
object TrajectoryFactory {
    private lateinit var carouselStartPose: Pose2d
    private lateinit var farParkStartPose: Pose2d
    private lateinit var closeParkStartPose: Pose2d
    private lateinit var hubFrontStartPose: Pose2d
    private lateinit var hubTopStartPose: Pose2d

    lateinit var startToHubFront: ParallelTrajectory
    lateinit var startToHubTop: ParallelTrajectory

    lateinit var startToCarousel: ParallelTrajectory

    lateinit var startToParkFar: ParallelTrajectory
    lateinit var startToParkClose: ParallelTrajectory

    lateinit var carouselToHubFront: ParallelTrajectory
    lateinit var carouselToHubBottom: ParallelTrajectory

    lateinit var hubFrontToPark: ParallelTrajectory
    lateinit var hubTopToParkIn: ParallelTrajectory
    lateinit var hubTopToParkOut: ParallelTrajectory
    lateinit var hubBottomToParkIn: ParallelTrajectory
    lateinit var hubBottomToParkOut: ParallelTrajectory

    lateinit var carouselToPark: ParallelTrajectory

    fun initializeTrajectories() {
        carouselStartPose = Pose2d(-36.0, 63.0.switchColor, (if (Constants.color == Constants.Color.BLUE) 180.0 else 90.0).switchColorAngle.toRadians)
        farParkStartPose = Pose2d(-36.0, 63.0.switchColor, 0.0.switchColorAngle.toRadians)
        closeParkStartPose = Pose2d(6.0, 63.0.switchColor, 0.0.switchColorAngle.toRadians)
        hubFrontStartPose = Pose2d(-12.0, 63.0.switchColor, 270.0.switchColorAngle.toRadians)
        hubTopStartPose = Pose2d(6.0, 63.0.switchColor, 270.0.switchColorAngle.toRadians)

        startToHubFront = MecanumDrive.trajectoryBuilder(hubFrontStartPose, hubFrontStartPose.heading)
            .forward(21.0)
            .build()
        startToHubTop = MecanumDrive.trajectoryBuilder(hubTopStartPose, hubFrontStartPose.heading)
            .splineToSplineHeading(Pose2d(6.0, 24.0.switchColor, 180.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
            .build()
        startToCarousel = if (Constants.color == Constants.Color.BLUE)
            MecanumDrive.trajectoryBuilder(carouselStartPose, carouselStartPose.heading)
                .splineToConstantHeading(Vector2d(-53.5, 62.0.switchColor), 175.0.switchColorAngle.toRadians)
                .build()
        else
            MecanumDrive.trajectoryBuilder(carouselStartPose, carouselStartPose.heading + 130.0.switchColorAngle.toRadians)
                .splineToSplineHeading(Pose2d(-56.5, 59.0.switchColor, 120.0.switchColorAngle.toRadians), 200.0.switchColorAngle.toRadians)
                .build()

        startToParkFar = MecanumDrive.trajectoryBuilder(farParkStartPose, farParkStartPose.heading - 4.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(40.0, 61.0.switchColor), 0.0.switchColorAngle.toRadians)
            .build()
        startToParkClose = MecanumDrive.trajectoryBuilder(closeParkStartPose, closeParkStartPose.heading - 10.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(40.0, 61.0.switchColor), 0.0.switchColorAngle.toRadians)
            .build()

        carouselToHubFront = MecanumDrive.trajectoryBuilder(startToCarousel.trajectory.end(), true)
            .splineToSplineHeading(Pose2d(-12.0, 42.0.switchColor, 270.0.switchColorAngle.toRadians), 320.0.switchColorAngle.toRadians)
            .build()
        carouselToHubBottom = if (Constants.color == Constants.Color.BLUE) MecanumDrive.trajectoryBuilder(startToCarousel.trajectory.end(), startToCarousel.trajectory.end().heading + 90.0.toRadians)
            .splineToSplineHeading(Pose2d(-30.0, 24.0.switchColor, 0.0.switchColorAngle.toRadians), 320.0.switchColorAngle.toRadians)
            .build()
        else
            MecanumDrive.trajectoryBuilder(startToCarousel.trajectory.end(), startToCarousel.trajectory.end().heading + 180.0.toRadians)
                .splineToSplineHeading(Pose2d(-30.0, 24.0.switchColor, 0.0.switchColorAngle.toRadians), 320.0.switchColorAngle.toRadians)
                .build()

        hubFrontToPark = MecanumDrive.trajectoryBuilder(carouselToHubFront.trajectory.end(), carouselToHubFront.trajectory.end().heading + 90.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(20.0, 42.0.switchColor, 180.0.switchColorAngle.toRadians), 0.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(40.0, 42.0.switchColor, 180.0.switchColorAngle.toRadians), 0.0.switchColorAngle.toRadians)
            .build()
        hubTopToParkIn = MecanumDrive.trajectoryBuilder(startToHubTop.trajectory.end(), startToHubTop.trajectory.end().heading - 90.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(6.0, 32.0.switchColor, 180.0.switchColorAngle.toRadians), 90.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(40.0, 40.0.switchColor), 0.0.switchColorAngle.toRadians)
            .build()
        hubTopToParkOut = MecanumDrive.trajectoryBuilder(startToHubTop.trajectory.end(), startToHubTop.trajectory.end().heading - 90.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(6.0, 53.0.switchColor, 180.0.switchColorAngle.toRadians), 90.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(40.0, 61.0.switchColor), 0.0.switchColorAngle.toRadians)
            .build()
        hubBottomToParkIn = MecanumDrive.trajectoryBuilder(carouselToHubBottom.trajectory.end(),
            carouselToHubBottom.trajectory.end().heading + 90.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(-30.0, 38.0.switchColor, 0.0.switchColorAngle.toRadians), 90.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(-12.0, 45.0.switchColor), 0.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(13.0, 40.0.switchColor), 0.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(40.0, 40.0.switchColor), 0.0.switchColorAngle.toRadians)
            .build()
        hubBottomToParkOut = MecanumDrive.trajectoryBuilder(startToCarousel.trajectory.end(), startToCarousel.trajectory.end().heading + 90.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(-30.0, 36.0.switchColor, 0.0.switchColorAngle.toRadians), 90.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(40.0, 61.0.switchColor), 0.0.switchColorAngle.toRadians)
            .build()

        carouselToPark = MecanumDrive.trajectoryBuilder(startToCarousel.trajectory.end(),
            startToCarousel.trajectory.end().heading + if (Constants.color == Constants.Color.BLUE) 180.0.switchColorAngle.toRadians else 225.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(40.0, 61.0.switchColor, 180.0.switchColorAngle.toRadians), 0.0.switchColorAngle.toRadians)
            .build()
    }

    val Double.toRadians get() = (Math.toRadians(this))
    val Double.switchColorAngle get () = (if (Constants.color == Constants.Color.BLUE) this else 360 - this)
    val Double.switchColor get () = (if (Constants.color == Constants.Color.BLUE) this else this * -1)
}