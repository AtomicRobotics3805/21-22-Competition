package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.*
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.switchColorAngle
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.toRadians
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay
import org.firstinspires.ftc.teamcode.util.commands.driving.TurnRelative
import org.firstinspires.ftc.teamcode.util.commands.parallel
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Suppress("Unused")
object AutoRoutines {
    val testRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.testTrajectory)
        }

    val parkCloseRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToParkClose)
        }
    val parkFarRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToParkFar)
        }

    val carouselRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
            +Carousel.fullRotation
            +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToPark)
        }

    val hubFrontRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToHubFront)
            //TODO: Deposit preloaded freight
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubFrontToPark)
        }
    val hubTopParkInRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToHubTop)
            //TODO: Deposit preloaded freight
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubTopToParkIn)
        }
    val hubTopParkOutRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToHubTop)
            //TODO: Deposit preloaded freight
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubTopToParkOut)
        }

    val carouselHubRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
            +Carousel.fullRotation
            +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubFront)
            +dropFreightRoutine
            +DeadWheelServo.up
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubFrontToPark)
        }

    val simpleCarouselHubRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Bucket.up
                +MecanumDrive.followTrajectory(TrajectoryFactory.simpleStartToHub1)
            }
            +MecanumDrive.turnRelative(-90.0.switchColorAngle.toRadians)
            +MecanumDrive.followTrajectory(TrajectoryFactory.simpleStartToHub2)
            +MecanumDrive.turnRelative(90.0.switchColorAngle.toRadians)
            +dropFreightRoutine
            +MecanumDrive.turnRelative((if (Constants.color == Constants.Color.BLUE) 80.0 else 80.0).switchColorAngle.toRadians)
            +MecanumDrive.followTrajectory(TrajectoryFactory.simpleHubToCarousel1)
            if (Constants.color == Constants.Color.BLUE) {
                +MecanumDrive.turnRelative(-52.0.switchColorAngle.toRadians)
                +MecanumDrive.followTrajectory(TrajectoryFactory.simpleHubToCarousel2)
                +Carousel.fullRotation
            }
            else {
                +Carousel.fullRotationReverse
                +MecanumDrive.turn(135.0.switchColorAngle.toRadians)
            }
            +MecanumDrive.followTrajectory(TrajectoryFactory.simpleCarouselToPark1)
            +MecanumDrive.turn(0.0.switchColorAngle.toRadians)
            +DeadWheelServo.upAutonomous
            +MecanumDrive.followTrajectory(TrajectoryFactory.simpleCarouselToPark2)
            +DeadWheelServo.resetTranslationalPID
        }

    val carouselHubBottomParkInRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
            +Carousel.fullRotation
            +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubBottom)
            //TODO: Deposit preloaded freight
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToParkIn)
        }
    val carouselHubBottomParkOutRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
            +Carousel.fullRotation
            +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubBottom)
            //TODO: Deposit preloaded freight
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToParkOut)
        }

    private val dropFreightRoutine: AtomicCommand
        get() = sequential {
            +Arm.moveArmAutonomous()
            +BucketLatch.open
            +Bucket.drop
            +Delay(1.6)
            +parallel {
                +Bucket.up
                +Arm.toStart
            }
        }
}