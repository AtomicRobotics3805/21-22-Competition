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
import org.firstinspires.ftc.teamcode.util.commands.other.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay
import org.firstinspires.ftc.teamcode.util.commands.parallel
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Suppress("Unused", "MemberVisibilityCouldBePrivate")
object FrankieRoutines {

    @JvmField
    var EXTEND_ROTATE_BUCKET_DELAY = 0.2
    @JvmField
    var EXTEND_INTAKE_RETRACT_LIFT_DELAY = 0.5
    @JvmField
    var OPEN_LOCK_DELAY = 1.0
    @JvmField
    var TRANSFER_DELAY = 1.0
    @JvmField
    var REVERSE_DELAY = .12

    val noCarouselFreightRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +sequential {
                    +Intake.Extender.instantExtend
                    +Delay(EXTEND_INTAKE_RETRACT_LIFT_DELAY)
                    +Lift.Extender.retractAtStart
                }
                +Bucket.Lock.close
                +Intake.Lock.close
            }
            +Lift.Swivel.ToPreloadPosition()
            +parallel {
                +sequential {
                    +Lift.Pivot.toPosition(shippingHubPosition)
                    +parallel {
                        +Lift.Extender.fullExtend
                        +sequential {
                            +Delay(EXTEND_ROTATE_BUCKET_DELAY)
                            +Bucket.Rotator.score
                        }
                    }
                }
                +Intake.Extender.retract
            }
            +Bucket.Lock.open
            +Delay(OPEN_LOCK_DELAY)
            +parallel {
                +Lift.Extender.retract
                +Bucket.Rotator.collect
                +Intake.Extender.extend
                +Bucket.Lock.close
            }
            +Lift.Swivel.idle
            +Lift.Pivot.toAngle(0.0)
            +Lift.Swivel.toCollect
            +parallel {
                +Bucket.Lock.collect
                +Lift.Extender.collect
                +Intake.Extender.partialExtend
                +Intake.Spinner.idle
                +MecanumDrive.followTrajectory(TrajectoryFactory.startToInsideWarehouse)
            }
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
                    +Intake.Spinner.reverse
                    +Delay(REVERSE_DELAY)
                    +Intake.Spinner.idle
                    +Intake.Extender.retract
                    +Intake.Spinner.start
                    +Intake.Lock.open
                    +Delay(TRANSFER_DELAY)
                    +Bucket.Lock.close
                    +Lift.Swivel.toHigh
                    +Intake.Spinner.stop
                    +sequential {
                        +Lift.Pivot.toPosition(shippingHubPosition)
                        +parallel {
                            +Lift.Extender.fullExtend
                            +sequential {
                                +Delay(EXTEND_ROTATE_BUCKET_DELAY)
                                +Bucket.Rotator.score
                            }
                        }
                    }
                }
                +MecanumDrive.followTrajectory(TrajectoryFactory.insideWarehouseToOutsideWarehouse)
            }
            +Bucket.Lock.open
            +Delay(OPEN_LOCK_DELAY)
            +parallel {
                +Lift.Extender.retract
                +Bucket.Rotator.collect
                //+Intake.Spinner.idle
                +Intake.Extender.extend
                +Bucket.Lock.close
                +Intake.Lock.close
            }
            +Lift.Swivel.idle
            +Lift.Pivot.toAngle(0.0)
            +Lift.Swivel.toCollect
            +parallel {
                +Bucket.Lock.collect
                +Lift.Extender.collect
                +Intake.Extender.partialExtend
                +MecanumDrive.followTrajectory(TrajectoryFactory.startToInsideWarehouse)
            }
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
            +Bucket.Lock.close
            +Lift.Swivel.toHigh
            +setTeleOpPosition
            +Lift.Pivot.toPosition(shippingHubPosition)
            +parallel {
                +Lift.Extender.fullExtend
                +sequential {
                    +Delay(EXTEND_ROTATE_BUCKET_DELAY)
                    +Bucket.Rotator.score
                }
            }
        }

    val collectAtStartTeleOpRoutine: AtomicCommand
        get() = sequential {
            +Lift.Extender.collect
            +parallel {
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
            +Lift.Pivot.toAngle(0.0)
            +parallel {
                +Lift.Swivel.toCollect
                +Intake.Lock.close
                +Bucket.Lock.collect
                +Bucket.Rotator.collect
            }
            +Lift.Extender.collect
        }

    val setTeleOpPosition: AtomicCommand
        get() = CustomCommand(_start = { MecanumDrive.poseEstimate = teleOpAutomaticDepositPosition })
}