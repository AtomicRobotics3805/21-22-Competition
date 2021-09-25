import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand

object ObjectDetection {
    enum class ObjectPosition {
        UNKNOWN,
        LEFT,
        MIDDLE,
        RIGHT
    }

    val position: ObjectPosition = ObjectPosition.UNKNOWN

    val detect: AtomicCommand
        get() = DetectCommand()

    class DetectCommand : AtomicCommand() {
        override val _isDone: Boolean
            get() = TODO()

        override fun start() {

        }

        override fun execute() {

        }

        override fun done(interrupted: Boolean) {

        }
    }
}