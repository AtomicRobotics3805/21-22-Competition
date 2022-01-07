package org.firstinspires.ftc.teamcode.subsystems.frankie.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.CustomGamepad
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.TimedCustomCommand
import org.firstinspires.ftc.teamcode.util.commands.subsystems.MotorToPosition
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import kotlin.math.abs

@Suppress("Unused", "MemberVisibilityCanBePrivate")
@Config
object Carousel : Subsystem {
    @JvmField
    var CAROUSEL_NAME = "carousel"

    @JvmField
    var CAROUSEL_DIRECTION = DcMotorSimple.Direction.REVERSE

    @JvmField
    var CAROUSEL_FULL_ROTATION_TIME = 2.0
    // used when doing a full automatic rotation
    @JvmField
    var CAROUSEL_AUTOMATIC_SPEED = 0.4
    // used when pressing a button to rotate the carousel manually
    @JvmField
    var CAROUSEL_MANUAL_SPEED = 0.55
    // multiplied by the value of a trigger when rotating the carousel manually
    @JvmField
    var CAROUSEL_TRIGGER_SPEED = 0.7

    var on = false

    val switch: AtomicCommand
        get() = if (on) stop else start
    val start: AtomicCommand
        get() = powerCarousel(
            if (Constants.color == Constants.Color.BLUE)
                CAROUSEL_MANUAL_SPEED else -CAROUSEL_MANUAL_SPEED
        )
    val stop: AtomicCommand
        get() = powerCarousel(0.0)
    val fullRotation: AtomicCommand
        get() = if (Constants.color == Constants.Color.BLUE)
            fullRotation(1) else fullRotation(-1)

    private lateinit var carouselMotor: CRServo

    fun initialize() {
        carouselMotor = opMode.hardwareMap.get(CRServo::class.java, CAROUSEL_NAME)
        carouselMotor.direction = CAROUSEL_DIRECTION
    }

    fun powerCarouselTrigger(trigger: CustomGamepad.Trigger) = CustomCommand(
        getDone = { !trigger.down },
        _execute = {
            carouselMotor.power = trigger.amount.toDouble() * CAROUSEL_TRIGGER_SPEED
        })

    fun powerCarousel(power: Double) = CustomCommand(_start = {
        carouselMotor.power = power
        on = power != 0.0
    })

    fun fullRotation(direction: Int) = TimedCustomCommand(
        time = CAROUSEL_FULL_ROTATION_TIME,
        _start = {
            carouselMotor.power = CAROUSEL_AUTOMATIC_SPEED * direction
        },
        _done = {
            carouselMotor.power = 0.0
        }
    )

}