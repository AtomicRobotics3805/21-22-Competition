package org.firstinspires.ftc.teamcode.util.commands.delays

import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand

class WaitUntil(private val check: () -> Boolean) : AtomicCommand() {
    override val _isDone: Boolean
        get() = check.invoke()
}