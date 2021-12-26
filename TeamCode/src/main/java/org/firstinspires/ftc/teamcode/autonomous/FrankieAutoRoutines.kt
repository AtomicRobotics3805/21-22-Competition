@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Bucket
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms.Lift
import org.firstinspires.ftc.teamcode.subsystems.trio.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay
import org.firstinspires.ftc.teamcode.util.commands.parallel
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Suppress("Unused", "MemberVisibilityCouldBePrivate")
object FrankieAutoRoutines {
    val shippingHubPosition = Vector2d(-12.0, 24.0)

    val noCarouselFreightRoutine: AtomicCommand
        get() = sequential {
            +Intake.Rotator.downStart
            +parallel {
                +sequential {
                    +Lift.Extender.ResetAtStart()
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
                +Intake.Extender.extend
            }
            +Bucket.Latch.open
            +parallel {
                +MecanumDrive.followTrajectory(TrajectoryFactory.startToWarehouse)
                +Lift.Pivot.toAngle(0.0)
                +Lift.Swivel.ToCollectCareful()
                +Intake.Spinner.start
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
                    +parallel {
                        +sequential {
                            +Lift.Swivel.toHigh
                            +Lift.Extender.fullExtend
                        }
                        +Lift.Pivot.toPosition(shippingHubPosition)
                        +Intake.Extender.extend
                        +Bucket.Rotator.drop
                    }
                }
                +MecanumDrive.followTrajectory(TrajectoryFactory.warehouseToHub)
            }
            +Bucket.Latch.open
            +parallel {
                +Lift.Pivot.toAngle(0.0)
                +Lift.Swivel.ToCollectCareful()
                +Intake.Spinner.start
                +Intake.Extender.extend
                +Intake.Rotator.down
                +MecanumDrive.followTrajectory(TrajectoryFactory.hubToWarehouse)
            }
        }

}