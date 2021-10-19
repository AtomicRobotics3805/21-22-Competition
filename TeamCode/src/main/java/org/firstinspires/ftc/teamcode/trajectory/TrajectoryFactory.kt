package org.firstinspires.ftc.teamcode.trajectory

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive

@Suppress("unused", "MemberVisibilityCanBePrivate")
@Config
object TrajectoryFactory {
    private val carouselStartPose = Pose2d(-36.0, 63.0.switchColor, (if (Constants.color == Constants.Color.BLUE) 180.0 else 90.0).switchColorAngle.toRadians)
    private val farParkStartPose = Pose2d(-36.0, 63.0.switchColor, 0.0.switchColorAngle.toRadians)
    private val closeParkStartPose = Pose2d(6.0, 63.0.switchColor, 0.0.switchColorAngle.toRadians)
    private val hubFrontStartPose = Pose2d(-12.0, 63.0.switchColor, 270.0.switchColorAngle.toRadians)
    private val hubTopStartPose = Pose2d(6.0, 63.0.switchColor, 270.0.switchColorAngle.toRadians)

    val startToHubFront = MecanumDrive.trajectoryBuilder(hubFrontStartPose, hubFrontStartPose.heading)
        .forward(21.0)
        .build()
    val startToHubTop = MecanumDrive.trajectoryBuilder(hubTopStartPose, hubFrontStartPose.heading)
        .splineToSplineHeading(Pose2d(6.0, 24.0.switchColor, 180.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
        .build()

    val startToCarouselBlue = MecanumDrive.trajectoryBuilder(carouselStartPose, carouselStartPose.heading)
        .splineToConstantHeading(Vector2d(-53.5, 62.0.switchColor), 175.0.switchColorAngle.toRadians)
        .build()
    val startToCarouselRed = MecanumDrive.trajectoryBuilder(carouselStartPose, carouselStartPose.heading + 130.0.switchColorAngle.toRadians)
        .splineToSplineHeading(Pose2d(-56.5, 59.0.switchColor, 120.0.switchColorAngle.toRadians), 200.0.switchColorAngle.toRadians)
        .build()

    val startToParkFar = MecanumDrive.trajectoryBuilder(farParkStartPose, farParkStartPose.heading - 4.0.switchColorAngle.toRadians)
        .splineToConstantHeading(Vector2d(40.0, 61.0.switchColor), 0.0.switchColorAngle.toRadians)
        .build()
    val startToParkClose = MecanumDrive.trajectoryBuilder(closeParkStartPose, closeParkStartPose.heading - 10.0.switchColorAngle.toRadians)
        .splineToConstantHeading(Vector2d(40.0, 61.0.switchColor), 0.0.switchColorAngle.toRadians)
        .build()

    val carouselToHubFront = MecanumDrive.trajectoryBuilder((if (Constants.color == Constants.Color.BLUE) startToCarouselBlue else startToCarouselRed).trajectory.end(), true)
        .splineToSplineHeading(Pose2d(-12.0, 42.0.switchColor, 270.0.switchColorAngle.toRadians), 320.0.switchColorAngle.toRadians)
        .build()
    val carouselToHubBottomBlue = MecanumDrive.trajectoryBuilder(startToCarouselBlue.trajectory.end(), startToCarouselBlue.trajectory.end().heading + 90.0.toRadians)
        .splineToSplineHeading(Pose2d(-30.0, 24.0.switchColor, 0.0.switchColorAngle.toRadians), 320.0.switchColorAngle.toRadians)
        .build()
    val carouselToHubBottomRed = MecanumDrive.trajectoryBuilder(startToCarouselRed.trajectory.end(), startToCarouselRed.trajectory.end().heading + 180.0.toRadians)
        .splineToSplineHeading(Pose2d(-30.0, 24.0.switchColor, 0.0.switchColorAngle.toRadians), 320.0.switchColorAngle.toRadians)
        .build()

    val hubFrontToPark = MecanumDrive.trajectoryBuilder(carouselToHubFront.trajectory.end(), carouselToHubFront.trajectory.end().heading + 90.0.switchColorAngle.toRadians)
        .splineToSplineHeading(Pose2d(20.0, 42.0.switchColor, 180.0.switchColorAngle.toRadians), 0.0.switchColorAngle.toRadians)
        .splineToSplineHeading(Pose2d(40.0, 42.0.switchColor, 180.0.switchColorAngle.toRadians), 0.0.switchColorAngle.toRadians)
        .build()
    val hubTopToParkIn = MecanumDrive.trajectoryBuilder(startToHubTop.trajectory.end(), startToHubTop.trajectory.end().heading - 90.0.switchColorAngle.toRadians)
        .splineToSplineHeading(Pose2d(6.0, 32.0.switchColor, 180.0.switchColorAngle.toRadians), 90.0.switchColorAngle.toRadians)
        .splineToConstantHeading(Vector2d(40.0, 40.0.switchColor), 0.0.switchColorAngle.toRadians)
        .build()
    val hubTopToParkOut = MecanumDrive.trajectoryBuilder(startToHubTop.trajectory.end(), startToHubTop.trajectory.end().heading - 90.0.switchColorAngle.toRadians)
        .splineToSplineHeading(Pose2d(6.0, 53.0.switchColor, 180.0.switchColorAngle.toRadians), 90.0.switchColorAngle.toRadians)
        .splineToConstantHeading(Vector2d(40.0, 61.0.switchColor), 0.0.switchColorAngle.toRadians)
        .build()
    val hubBottomToParkIn = MecanumDrive.trajectoryBuilder((if (Constants.color == Constants.Color.BLUE) carouselToHubBottomBlue else carouselToHubBottomRed).trajectory.end(),
        (if (Constants.color == Constants.Color.BLUE) carouselToHubBottomBlue else carouselToHubBottomRed).trajectory.end().heading + 90.0.switchColorAngle.toRadians)
        .splineToSplineHeading(Pose2d(-30.0, 38.0.switchColor, 0.0.switchColorAngle.toRadians), 90.0.switchColorAngle.toRadians)
        .splineToConstantHeading(Vector2d(-12.0, 45.0.switchColor), 0.0.switchColorAngle.toRadians)
        .splineToConstantHeading(Vector2d(13.0, 40.0.switchColor), 0.0.switchColorAngle.toRadians)
        .splineToConstantHeading(Vector2d(40.0, 40.0.switchColor), 0.0.switchColorAngle.toRadians)
        .build()
    val hubBottomToParkOut = MecanumDrive.trajectoryBuilder((if (Constants.color == Constants.Color.BLUE) carouselToHubBottomBlue else carouselToHubBottomRed).trajectory.end(),
        (if (Constants.color == Constants.Color.BLUE) carouselToHubBottomBlue else carouselToHubBottomRed).trajectory.end().heading + 90.0.switchColorAngle.toRadians)
        .splineToSplineHeading(Pose2d(-30.0, 36.0.switchColor, 0.0.switchColorAngle.toRadians), 90.0.switchColorAngle.toRadians)
        .splineToConstantHeading(Vector2d(40.0, 61.0.switchColor), 0.0.switchColorAngle.toRadians)
        .build()

    val carouselToPark = MecanumDrive.trajectoryBuilder((if (Constants.color == Constants.Color.BLUE) startToCarouselBlue else startToCarouselRed).trajectory.end(), (if (Constants.color == Constants.Color.BLUE)
        startToCarouselBlue.trajectory.end().heading + 180.0.switchColorAngle.toRadians else startToCarouselRed.trajectory.end().heading + 225.0.switchColorAngle.toRadians))
        .splineToSplineHeading(Pose2d(40.0, 61.0.switchColor, 180.0.switchColorAngle.toRadians), 0.0.switchColorAngle.toRadians)
        .build()

    val Double.toRadians get() = (Math.toRadians(this))
    val Double.switchColorAngle get () = (if (Constants.color == Constants.Color.BLUE) this else 360 - this)
    val Double.switchColor get () = (if (Constants.color == Constants.Color.BLUE) this else this * -1)
}