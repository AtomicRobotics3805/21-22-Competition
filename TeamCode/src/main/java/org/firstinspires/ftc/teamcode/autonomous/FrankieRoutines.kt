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
                +sequential {
                    +Intake.Extender.extend
                    +Delay(0.5)
                    +Lift.Extender.retractAtStart
                    +Lift.Extender.idle
                }
                +Bucket.Latch.close
                +Intake.Lock.close
            }
            //+Lift.Extender.ResetAtStart()
            +Lift.Swivel.toPreloadPosition
            +Lift.Pivot.toPosition(shippingHubPosition)
            +parallel {
                +Lift.Extender.fullExtend
                +sequential {
                    +Delay(0.2)
                    +Bucket.Rotator.score
                }
            }
            +Bucket.Latch.open
            +Delay(1.0)
            +parallel {
                +Lift.Pivot.toAngle(0.0)
                +Lift.Extender.retract
                +Bucket.Rotator.collect
                +Intake.Spinner.start
                +Intake.Extender.extend
            }
            +Lift.Extender.idle
            +Lift.Swivel.toCollect
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToInsideWarehouse)
            +deliverFreightRoutine
            /*+deliverFreightRoutine
            +deliverFreightRoutine
            +deliverFreightRoutine
            +deliverFreightRoutine*/
        }

    val deliverFreightRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +sequential {
                    +parallel {
                        +Intake.Extender.retract
                        +Intake.Spinner.idle
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
                            +Bucket.Rotator.score
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
                +Lift.Extender.retract
                +Intake.Extender.extend
                +Bucket.Rotator.collect
                +MecanumDrive.followTrajectory(TrajectoryFactory.outsideWarehouseToInsideWarehouse)
            }
        }

    val transferMoveLiftTeleOpRoutine: AtomicCommand
        get() = sequential {
            +toTransfer
            +Delay(1.0)
            +moveLiftTeamHub
        }

    val toTransfer: AtomicCommand
        get() = sequential {
            +Lift.Extender.collect
            +parallel {
                +Intake.Extender.fullRetract
                +Intake.Spinner.idle
            }
            +parallel {
                +Intake.Lock.open
                +Intake.Spinner.start
            }
        }

    val moveLiftTeamHub: AtomicCommand
        get() = sequential {
            +Intake.Spinner.idle
            +Bucket.Latch.close
            +Lift.Swivel.toHigh
            +sequential {
                +setTeleOpPosition
                +Lift.Pivot.toPosition(shippingHubPosition)
            }
            +Bucket.Rotator.score
        }

    val collectAtStartTeleOpRoutine: AtomicCommand
        get() = sequential {
            +Lift.Extender.collect
            +Lift.Extender.idle
            +parallel {
                +Intake.Spinner.start
                +Intake.Extender.extend
                +Intake.Lock.close
                +Bucket.Latch.collect
            }
            +Bucket.Rotator.collect
        }

    val dropAndCollectTeleOpRoutine: AtomicCommand
        get() = sequential {
            +Bucket.Latch.open
            +Delay(0.3)
            +Lift.Extender.retract
            +Lift.Pivot.toAngle(0.0)
            +parallel {
                +Lift.Swivel.toCollect
                +Intake.Spinner.start
                +Intake.Extender.extend
                +Intake.Lock.close
                +Bucket.Latch.collect
            }
            +Bucket.Rotator.collect
            +Lift.Extender.collect
        }

    val setTeleOpPosition: AtomicCommand
        get() = CustomCommand(_start = { MecanumDrive.poseEstimate = teleOpAutomaticDepositPosition })
}