package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Suppress("MemberVisibilityCanBePrivate")
class CommandGamepad(qualcommGamepad: Gamepad) {

    enum class PromptType {
        DOWN,
        PRESSED,
        RELEASED
    }

    val gamepad = CustomGamepad(qualcommGamepad)

    val a = CommandButton(gamepad.a)
    val b = CommandButton(gamepad.b)
    val x = CommandButton(gamepad.x)
    val y = CommandButton(gamepad.y)

    val dpadUp = CommandButton(gamepad.dpadUp)
    val dpadDown = CommandButton(gamepad.dpadDown)
    val dpadLeft = CommandButton(gamepad.dpadLeft)
    val dpadRight = CommandButton(gamepad.dpadRight)

    val leftBumper = CommandButton(gamepad.leftBumper)
    val rightBumper = CommandButton(gamepad.rightBumper)

    val leftTrigger = CommandTrigger(gamepad.leftTrigger)
    val rightTrigger = CommandTrigger(gamepad.rightTrigger)

    val leftStickButton = CommandButton(gamepad.leftStick.button)
    val rightStickButton = CommandButton(gamepad.rightStick.button)

    private val buttons: List<CommandButton> = listOf(a, b, x, y, dpadUp, dpadDown, dpadLeft,
        dpadRight, leftBumper, rightBumper, leftStickButton, rightStickButton)
    private val triggers: List<CommandTrigger> = listOf(leftTrigger, rightTrigger)

    fun update() {
        gamepad.update()
        buttons.forEach { it.update() }
        triggers.forEach { it.update() }
    }

    class CommandButton(val button: CustomGamepad.Button) {
        val down = CommandButtonPrompt(button, PromptType.DOWN)
        val pressed = CommandButtonPrompt(button, PromptType.PRESSED)
        val released = CommandButtonPrompt(button, PromptType.RELEASED)

        fun update() {
            down.update()
            pressed.update()
            released.update()
        }

        class CommandButtonPrompt(private val button: CustomGamepad.Button,
                                   private val promptType: PromptType) {
            var command: (() -> AtomicCommand)? = null

            fun update() {
                if(command != null && ((promptType == PromptType.DOWN && button.down) ||
                                (promptType == PromptType.PRESSED && button.pressed) || 
                                (promptType == PromptType.RELEASED && button.released))) {
                    CommandScheduler.commandsToSchedule += command!!.invoke()
                }
            }
        }
    }
    
    class CommandTrigger (val trigger: CustomGamepad.Trigger) {
        val down = CommandTriggerPrompt(trigger, PromptType.DOWN)
        val pressed = CommandTriggerPrompt(trigger, PromptType.PRESSED)
        val released = CommandTriggerPrompt(trigger, PromptType.RELEASED)

        fun update() {
            down.update()
            pressed.update()
            released.update()
        }

        class CommandTriggerPrompt(private val trigger: CustomGamepad.Trigger,
                                   private val promptType: PromptType) {
            var command: (() -> AtomicCommand)? = null

            fun update() {
                if(command != null && ((promptType == PromptType.DOWN && trigger.down) ||
                            (promptType == PromptType.PRESSED && trigger.pressed) ||
                            (promptType == PromptType.RELEASED && trigger.released))) {
                    CommandScheduler.commandsToSchedule += command!!.invoke()
                }
            }
        }
    }
}