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
    private val startPose = Pose2d()

    val carouselRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
            +Carousel.fullRotation
            +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToPark)
        }

    val hubRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
            +Carousel.fullRotation
            +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHub)
            +Arm.toHigh
            +Bucket.drop
            +parallel {
                +Bucket.up
                +MecanumDrive.followTrajectory(TrajectoryFactory.hubToPark)
            }
        }

    val parkRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToPark)
        }
}