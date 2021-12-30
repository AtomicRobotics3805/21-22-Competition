@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcode.autonomous

import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Lift
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.shippingHubPosition
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.teleOpAutomaticDepositPosition
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay
import org.firstinspires.ftc.teamcode.util.commands.parallel
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Suppress("Unused", "MemberVisibilityCouldBePrivate")
object FrankieRoutines {

    val noCarouselFreightRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Intake.Rotator.downStart
                +Lift.Extender.retract
            }
            +parallel {
                +sequential {
                    //+Lift.Extender.ResetAtStart()
                    +Lift.Swivel.toPreloadPosition
                    +parallel {
                        +Lift.Pivot.toPosition(shippingHubPosition)
                        +Lift.Extender.fullExtend
                        +sequential {
                            +Delay(0.2)
                            +Bucket.Rotator.drop
                        }
                    }
                }
            }
            +Bucket.Latch.open
            +Delay(0.3)
            +parallel {
                +MecanumDrive.followTrajectory(TrajectoryFactory.startToInsideWarehouse)
                +Lift.Pivot.toAngle(0.0)
                +Lift.Swivel.ToCollectCareful()
                +Intake.Spinner.start
                +Intake.Extender.extend
            }
            +deliverFreightRoutine
            +deliverFreightRoutine
            +deliverFreightRoutine
            +deliverFreightRoutine
            +deliverFreightRoutine
        }

    val deliverFreightRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +sequential {
                    +parallel {
                        +Intake.Extender.retract
                        +Intake.Rotator.up
                        +Intake.Spinner.idle
                        +Bucket.Rotator.up
                    }
                    +Intake.Spinner.start
                    +Delay(0.4)
                    +Bucket.Latch.close
                    +Lift.Swivel.toHigh
                    +parallel {
                        +Lift.Extender.fullExtend
                        +Lift.Pivot.toPosition(shippingHubPosition)
                        +sequential {
                            +Delay(0.2)
                            +Bucket.Rotator.drop
                        }
                    }
                }
                +MecanumDrive.followTrajectory(TrajectoryFactory.insideWarehouseToOutsideWarehouse)
            }
            +Bucket.Latch.open
            +Delay(0.3)
            +parallel {
                +Lift.Pivot.toAngle(0.0)
                +Lift.Swivel.ToCollectCareful()
                +Intake.Extender.extend
                +Intake.Rotator.down
                +MecanumDrive.followTrajectory(TrajectoryFactory.outsideWarehouseToInsideWarehouse)
            }
        }

    val transferMoveLiftTeleOpRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Intake.Extender.retract
                +Intake.Rotator.up
                +Intake.Spinner.idle
                +Bucket.Rotator.up
            }
            +Intake.Spinner.start
            +Delay(0.4)
            +Bucket.Latch.close
            +Lift.Swivel.toHigh
            +parallel {
                +Lift.Extender.fullExtend
                +sequential {
                    +setTeleOpPosition
                    +Lift.Pivot.toPosition(shippingHubPosition)
                }
                +sequential {
                    +Delay(0.2)
                    +Bucket.Rotator.drop
                }
            }
        }

    val dropAndCollectTeleOpRoutine: AtomicCommand
        get() = sequential {
            +Bucket.Latch.open
            +Delay(0.3)
            +parallel {
                +Lift.Pivot.toAngle(0.0)
                +Lift.Swivel.ToCollectCareful()
                +Lift.Extender.retract
                +Intake.Spinner.start
                +Intake.Extender.extend
                +Intake.Rotator.down
            }
        }

    val setTeleOpPosition: AtomicCommand
        get() = CustomCommand(_start = { MecanumDrive.poseEstimate = teleOpAutomaticDepositPosition })
}