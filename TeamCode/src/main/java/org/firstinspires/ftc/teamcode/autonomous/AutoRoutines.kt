package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Arm
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Carousel
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.parallel
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Suppress("Unused")
object AutoRoutines {
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
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToHubTop)
            //TODO: Deposit preloaded freight
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubTopToParkOut)
        }
}