@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Lift
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.shippingHubPosition
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.teleOpAutomaticDepositPosition
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.other.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay
import org.firstinspires.ftc.teamcode.util.commands.parallel
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Suppress("Unused", "MemberVisibilityCouldBePrivate")
@Config
object FrankieRoutines {

    @JvmField
    var EXTEND_ROTATE_BUCKET_DELAY = 0.2
    @JvmField
    var EXTEND_INTAKE_RETRACT_LIFT_DELAY = 0.3
    @JvmField
    var OPEN_LOCK_DELAY = 1.0
    @JvmField
    var CLOSE_LOCK_DELAY = 0.3
    @JvmField
    var TRANSFER_DELAY = 1.0
    @JvmField
    var REVERSE_DELAY = .14
    @JvmField
    var EXTEND_INTAKE_PIVOT_DELAY = 1.0
    @JvmField
    var SWIVEL_TO_PIVOT_HEIGHT_DELAY = 0.5

    val noCarouselFreightRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +sequential {
                    +Intake.Extender.extend.i
                    +Delay(EXTEND_INTAKE_RETRACT_LIFT_DELAY)
                    +Lift.Extender.retractAtStart
                }
                +Bucket.Lock.close
                +Intake.Lock.close
            }
            +parallel {
                +Lift.Swivel.toPreloadPosition
                +sequential {
                    +Delay(EXTEND_INTAKE_PIVOT_DELAY)
                    // retracting intake to be completely outside the warehouse when depositing
                    +Intake.Extender.retract
                }
                +sequential {
                    +Lift.Swivel.upPivotHeightDelay
                    +parallel {
                        +Lift.Extender.fullExtend
                        +sequential {
                            +Delay(EXTEND_ROTATE_BUCKET_DELAY)
                            +Bucket.Rotator.score
                        }
                        +sequential {
                            +parallel {
                                +Lift.Extender.extendOpenLatchDelay
                                +Lift.Pivot.toPosition(shippingHubPosition)
                            }
                            +Bucket.Lock.open
                        }
                    }
                }
            }
            +dropFreightAndGoToWarehouse
            +deliverFreightRoutine
            +deliverFreightRoutine
            +deliverFreightRoutine
            //+deliverFreightRoutine
            //+deliverFreightRoutine
        }

    val deliverFreightRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +sequential {
                    //+Intake.Spinner.reverse
                    +Delay(REVERSE_DELAY)
                    //+Intake.Spinner.idle
                    +Intake.Extender.retract
                    //+Intake.Spinner.start
                    +Intake.Lock.open
                    +Delay(TRANSFER_DELAY)
                    +Bucket.Lock.close
                    +parallel {
                        +Lift.Swivel.toHigh
                        +Lift.Extender.retractAtStart
                        //+Intake.Spinner.stop
                        +sequential {
                            // extending intake to avoid hitting the bars when the lift pivots
                            +Intake.Extender.extend.i
                            +Delay(EXTEND_INTAKE_PIVOT_DELAY)
                            // retracting intake to be completely outside the warehouse when depositing
                            +Intake.Extender.retract
                        }
                        +sequential {
                            +Lift.Swivel.upPivotHeightDelay
                            +parallel {
                                +Lift.Pivot.toPosition(shippingHubPosition, TrajectoryFactory.outsideWarehouseStartPose)
                                +Lift.Extender.fullExtend
                                +sequential {
                                    +Delay(EXTEND_ROTATE_BUCKET_DELAY)
                                    +Bucket.Rotator.score
                                }
                                +sequential {
                                    +Lift.Extender.extendOpenLatchDelay
                                    +Bucket.Lock.open
                                }
                            }
                        }
                    }
                }
                +MecanumDrive.followTrajectory(TrajectoryFactory.insideWarehouseToOutsideWarehouse)
            }
            +dropFreightAndGoToWarehouse
        }

    val dropFreightAndGoToWarehouse: AtomicCommand
        get() = parallel {
            +sequential {
                +parallel {
                    //+Intake.Spinner.idle
                    +Intake.Lock.close
                    +Intake.Extender.extend
                    +Bucket.Rotator.collect
                    +Lift.Extender.retract
                    +Lift.Pivot.toAngle(0.0)
                    +sequential {
                        +Delay(CLOSE_LOCK_DELAY)
                        +Bucket.Lock.close
                    }
                    +sequential {
                        +Delay(SWIVEL_TO_PIVOT_HEIGHT_DELAY)
                        +Lift.Swivel.toPivotHeight
                    }
                }
                +Lift.Swivel.toCollect
                +Lift.Extender.collect
                +parallel {
                    +Bucket.Lock.collect
                    +Intake.Extender.partialExtend
                }
            }
            +MecanumDrive.followTrajectory(TrajectoryFactory.startToInsideWarehouse)
        }

    val transferMoveLiftTeleOpRoutine: AtomicCommand
        get() = sequential {
            +toTransfer
            +Delay(TRANSFER_DELAY)
            +moveLiftTeamHub
        }

    val toTransfer: AtomicCommand
        get() = sequential {
            +Lift.Extender.collect
            +parallel {
                +Intake.Extender.retract
                //+Intake.Spinner.idle
            }
            +parallel {
                +Intake.Lock.open
                //+Intake.Spinner.start
            }
        }

    val moveLiftTeamHub: AtomicCommand
        get() = sequential {
            +Lift.Extender.idle
            +Bucket.Lock.close
            +Lift.Swivel.toHigh
            +Lift.Pivot.toPosition(shippingHubPosition, teleOpAutomaticDepositPosition)
            +parallel {
                +Lift.Extender.fullExtend
                +sequential {
                    +Delay(EXTEND_ROTATE_BUCKET_DELAY)
                    +Bucket.Rotator.score
                }
            }
        }

    val resetToCollectTeleOpRoutine: AtomicCommand
        get() = sequential {
            if (Lift.Extender.extensionMotor.currentPosition > Lift.Extender.COLLECT_DISTANCE *
                Lift.Extender.EXTENDER_TICKS_PER_INCH) +Lift.Extender.retract
            +Lift.Pivot.toAngle(0.0)
            +Lift.Swivel.toCollect
            +parallel {
                +Lift.Extender.collect
                +Intake.Lock.close
                +Bucket.Lock.collect
                +Bucket.Rotator.collect
            }
        }

    val dropAndCollectTeleOpRoutine: AtomicCommand
        get() = sequential {
            +Bucket.Lock.open
            +Delay(OPEN_LOCK_DELAY)
            +Lift.Extender.retract
            +Bucket.Lock.collect.i
            +Lift.Pivot.toAngle(0.0)
            +parallel {
                +Lift.Swivel.toCollect
                +Intake.Lock.close
                +Bucket.Rotator.collect
            }
            +Lift.Extender.collect
        }

    val setTeleOpPosition: AtomicCommand
        get() = CustomCommand(_start = { MecanumDrive.poseEstimate = teleOpAutomaticDepositPosition })
}