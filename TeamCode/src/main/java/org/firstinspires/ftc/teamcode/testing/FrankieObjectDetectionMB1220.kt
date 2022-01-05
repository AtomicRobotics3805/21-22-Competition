import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.Constants
import com.qualcomm.robotcore.util.ElapsedTime

object FrankieObjectDetectionMB1220 {
    enum class Position {
        UNKNOWN,
        LEFT,
        MIDDLE,
        RIGHT
    }
    enum class Barcode {
        FULL,
        EMPTY,
        UNKNOWN
    }

    var play = false;
    var threshold = 0.0 //Modify after testing for slim chassis.
    var position: Position = Position.UNKNOWN
    var barcodeA: Barcode = Barcode.UNKNOWN
    var barcodeB: Barcode = Barcode.UNKNOWN

    private lateinit var mb1220A: AnalogInput
    private lateinit var mb1220B: AnalogInput

    var mb1220AName = "mb1220a"
    var mb1220BName = "mb1220b"

    fun initialize() {

        mb1220A = Constants.opMode.hardwareMap.get(AnalogInput::class.java, mb1220AName)
        mb1220B = Constants.opMode.hardwareMap.get(AnalogInput::class.java, mb1220BName)
        position = Position.UNKNOWN
    }

    val detect: AtomicCommand
        get() = DetectCommand()

    class DetectCommand : AtomicCommand() {
        override val _isDone: Boolean
            get() = position != Position.UNKNOWN

        private val timer = ElapsedTime()

        override fun start(){
            var play = false
            position = Position.UNKNOWN
            timer.reset()
        }

        override fun execute(){

            while (play == false){

                if (mb1220A.voltage < threshold){
                    barcodeA = Barcode.FULL
                }
                else {
                    barcodeA = Barcode.EMPTY
                }

                timer.reset()

                while (timer.seconds() < 2){
                    //Do nothing.
                }

                if (mb1220B.voltage < threshold){
                    barcodeB = Barcode.FULL
                }
                else{
                    barcodeB = Barcode.EMPTY
                }
                timer.reset()

                while (timer.seconds() < 2){
                    //Do nothing.
                }
            }

            if (barcodeA == Barcode.FULL){
                position = Position.RIGHT
            }
            else if (barcodeB == Barcode.FULL){
                position = Position.MIDDLE
            }
            else{
                position = Position.LEFT
            }
        }
    }
}