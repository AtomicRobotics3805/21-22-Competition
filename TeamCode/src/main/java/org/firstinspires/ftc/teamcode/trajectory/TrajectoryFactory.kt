package org.firstinspires.ftc.teamcode.trajectory

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.Constants.startPose
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.toRadians
import org.firstinspires.ftc.teamcode.util.trajectories.a
import org.firstinspires.ftc.teamcode.util.trajectories.flip
import org.firstinspires.ftc.teamcode.util.trajectories.y

@Suppress("unused", "MemberVisibilityCanBePrivate")
@Config
object TrajectoryFactory