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

    var color = Color.BLUE
    val drive = MecanumDrive
    lateinit var startPose: Pose2d
    lateinit var opMode: OpMode
}