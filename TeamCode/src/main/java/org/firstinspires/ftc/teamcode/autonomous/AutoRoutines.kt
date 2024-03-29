package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.*
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.switchColorAngle
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.toRadians
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay
import org.firstinspires.ftc.teamcode.util.commands.other.CustomCommand
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
            +parallel {
                +Bucket.up
                +CapArm.up
                +OdometryServo.down
                +MecanumDrive.followTrajectory(TrajectoryFactory.startToHubTop)
            }
            +dropFreightRoutine
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubTopToWarehouseIn)
            +OdometryServo.upAutonomous
            +MecanumDrive.followTrajectory(TrajectoryFactory.warehouseInToParkIn)
            +OdometryServo.resetTranslationalPID
        }
    val hubTopParkOutRoutine: AtomicCommand
        get() = sequential {
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToHubTop)
            //TODO: Deposit preloaded freight
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubTopToParkOut)
        }

    val carouselHubRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Bucket.up
                +CapArm.up
                +OdometryServo.down
                +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
            }
            +Carousel.fullRotation
            +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubFront)
            +dropFreightRoutine
            +OdometryServo.upAutonomous
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubFrontToPark)
            +OdometryServo.resetTranslationalPID
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
            +MecanumDrive.turnRelative((if (Constants.color == Constants.Color.BLUE) 84.0 else 80.0).switchColorAngle.toRadians)
            +MecanumDrive.followTrajectory(TrajectoryFactory.simpleHubToCarousel1)
            if (Constants.color == Constants.Color.BLUE) {
                +MecanumDrive.turnRelative(-54.0.switchColorAngle.toRadians)
                +MecanumDrive.followTrajectory(TrajectoryFactory.simpleHubToCarousel2)
                +Carousel.fullRotation
            }
            else {
                +Carousel.fullRotationReverse
                +MecanumDrive.turn(135.0.switchColorAngle.toRadians)
            }
            +MecanumDrive.followTrajectory(TrajectoryFactory.simpleCarouselToPark1)
            //+Cap.idle
            +MecanumDrive.turn((if (Constants.color == Constants.Color.BLUE) 182.0 else 182.0).switchColorAngle.toRadians)
            +OdometryServo.upAutonomous
            +MecanumDrive.followTrajectory(TrajectoryFactory.simpleCarouselToPark2)
            +OdometryServo.resetTranslationalPID
        }

    val carouselHubBottomParkInRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Bucket.up
                +CapArm.up
                +OdometryServo.down
                +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
            }
            +Carousel.fullRotation
            +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubBottom)
            +dropFreightRoutine
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToWarehouse)
            +OdometryServo.upAutonomous
            +MecanumDrive.followTrajectory(TrajectoryFactory.warehouseToParkIn)
            +OdometryServo.resetTranslationalPID
        }
    val carouselHubBottomDuckStorageUnitRoutine: AtomicCommand
        get() = parallel {
            +ContainerSensor.detect
            +sequential {
                +Bucket.up.i
                +CapArm.up.i
                +BucketLock.open.i
                +OdometryServo.down.i
                if (Constants.color == Constants.Color.RED) {
                    +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
                    +Carousel.fullRotation
                    +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubBottom)
                    +dropFreightRoutineFast
                    +Intake.intakeIfEmpty
                    +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToDucks)
                    +MecanumDrive.followTrajectory(TrajectoryFactory.ducksToHubBottom)
                }
                else {
                    +MecanumDrive.followTrajectory(TrajectoryFactory.startToHubBottom)
                    +dropFreightRoutineFast
                    +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToCarousel)
                    +Intake.intakeIfEmpty
                    +Carousel.fullRotation
                    +Delay(0.8)
                    +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubBottom)
                }
                +Intake.stop
                +dropDuckRoutine
                +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToStorageUnit)
            }
        }
    val carouselHubBottomDuckWarehouseMiddleRoutine: AtomicCommand
        get() = parallel {
            +ContainerSensor.detect
            +sequential {
                +Bucket.up.i
                +CapArm.up.i
                +BucketLock.open.i
                +OdometryServo.down.i
                if (Constants.color == Constants.Color.RED) {
                    +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
                    +Carousel.fullRotation
                    +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubBottom)
                    +dropFreightRoutineFast
                    +Intake.intakeIfEmpty
                    +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToDucks)
                    +MecanumDrive.followTrajectory(TrajectoryFactory.ducksToHubBottom)
                } else {
                    +MecanumDrive.followTrajectory(TrajectoryFactory.startToHubBottom)
                    +dropFreightRoutineFast
                    +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToCarousel)
                    +Intake.intakeIfEmpty
                    +Carousel.fullRotation
                    +Delay(0.8)
                    +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubBottom)
                }
                +Intake.stop
                +dropDuckRoutine
                +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToWarehouseMiddle)
                +parallel {
                    +OdometryServo.upAutonomous
                    +CustomCommand(_start = { MecanumDrive.setDrivePower(Pose2d(-1.0, 0.0, 0.0)) })
                    +Delay(1.0)
                }
                +CustomCommand(_start = { MecanumDrive.setMotorPowers(0.5, -0.5, 0.5, -0.5) })
                +Delay(0.3)
                +CustomCommand(_start = { MecanumDrive.setDrivePower(Pose2d(0.0, 0.0, 0.0)) })
                +OdometryServo.resetTranslationalPID
            }
        }
    val carouselHubBottomParkOutRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Bucket.up
                +CapArm.up
                +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
            }
            +Carousel.fullRotation
            +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubBottom)
            +dropFreightRoutine
            +OdometryServo.upAutonomous
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToParkOut)
            +OdometryServo.resetTranslationalPID
        }

    val carouselHubBottomStorageUnitRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Bucket.up
                +CapArm.up
                +OdometryServo.down
                +MecanumDrive.followTrajectory(TrajectoryFactory.startToCarousel)
            }
            +Carousel.fullRotation
            +MecanumDrive.followTrajectory(TrajectoryFactory.carouselToHubBottom)
            +dropFreightRoutine
            +MecanumDrive.followTrajectory(TrajectoryFactory.hubBottomToStorageUnit)
        }

    private val dropFreightRoutine: AtomicCommand
        get() = sequential {
            +Arm.moveArmAutonomous()
            +Bucket.drop
            +Delay(1.0)
            +parallel {
                +Bucket.up
                +Arm.toStart
            }
        }

    private val dropFreightRoutineFast: AtomicCommand
        get() = sequential {
            +Arm.moveArmAutonomous()
            +Bucket.drop
            +Delay(1.0)
            +Bucket.up.i
            +Arm.toStart.i
        }

    private val dropDuckRoutine: AtomicCommand
        get() = sequential {
            +Arm.toHigh
            +Bucket.drop
            +Delay(1.5)
            +Bucket.up.i
            +Arm.toStart.i
        }
}