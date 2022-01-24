package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive

@Suppress("unused")
object Constants {
    enum class Color {
        BLUE,
        RED
    }
    enum class ObjectPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    var color = Color.BLUE
    val drive = MecanumDrive
    var startPose = Pose2d()
    var objectPosition = ObjectPosition.RIGHT

    lateinit var opMode: OpMode
}