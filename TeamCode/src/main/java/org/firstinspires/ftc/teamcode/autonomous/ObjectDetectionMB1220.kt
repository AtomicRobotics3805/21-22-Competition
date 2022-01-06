package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.switchColor
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.switchColorAngle
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory.toRadians
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.trajectories.ParallelTrajectory

@Config
object ObjectDetectionMB1220 {
    enum class Position {
        UNKNOWN,
        LEFT,
        MIDDLE,
        RIGHT
    }

    @JvmField
    var rightPosition = 0.18
    @JvmField
    var middlePosition = 0.34
    @JvmField
    var minVoltage = 0.07
    var position: Position = Position.UNKNOWN

    private lateinit var servo: Servo
    private lateinit var mb1220: AnalogInput

    var servoName = "detectionServo"
    var mb1220Name =  "mb1220"

    fun initialize() {
        servo = Constants.opMode.hardwareMap.get(Servo::class.java, servoName)
        mb1220 = Constants.opMode.hardwareMap.get(AnalogInput::class.java, mb1220Name)
        position = Position.UNKNOWN
    }

    class DetectCommand : AtomicCommand() {
        override val _isDone: Boolean
            get() = position != Position.UNKNOWN

        private var triedRight = false
        private val timer = ElapsedTime()

        override fun start() {
            position = Position.UNKNOWN
            timer.reset()
            servo.position = rightPosition
        }

        override fun execute() {
            if (timer.seconds() > 1.0 && position == Position.UNKNOWN) {
                timer.reset()
                if (!triedRight) {
                    triedRight = true
                    if (mb1220.voltage < minVoltage)
                        position = Position.RIGHT
                    else {
                        servo.position = middlePosition
                    }
                }
                else {
                    if (mb1220.voltage < minVoltage) {
                        position = Position.MIDDLE
                        TrajectoryFactory.startToHubTop = MecanumDrive.trajectoryBuilder(TrajectoryFactory.hubTopStartPose, TrajectoryFactory.hubTopStartPose.heading + 255.0.switchColorAngle.toRadians)
                                .splineToSplineHeading(Pose2d(7.0, 24.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
                                .build()
                    }
                    else position = Position.LEFT
                }
                Constants.opMode.telemetry.addData("Voltage", mb1220.voltage)
                Constants.opMode.telemetry.addData("Position", position)
                Constants.opMode.telemetry.update()
            }
        }
    }
}