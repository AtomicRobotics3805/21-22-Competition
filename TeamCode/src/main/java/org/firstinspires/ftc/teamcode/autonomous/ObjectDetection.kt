import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand

object ObjectDetection {
    enum class ObjectPosition {
        UNKNOWN,
        LEFT,
        MIDDLE,
        RIGHT,
        INVALID
    }

    private const val TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite"
    private val LABELS = arrayOf(
        "Ball",
        "Cube",
        "Duck",
        "Marker"
    )

    private const val VUFORIA_KEY = " AZ2jk6P/////AAABmck8NCyjWkCGvLdpx9HZ1kxI2vQPDlzN9vJnqy69nXRjvoXgBCEWZasRnd1hFjBpRiSXw4G4JwDFsk3kNSVko2UkuCgbi/RsiODF76MtldIi6YZGfrRMZTICMKwTanuOysh4Cn9Xd9nZzCpDiLAPLsUtKoj/DdBUn0gJuARMglUPW7/qirgtk0xI232ttZpXhgh9ya8R8LxnH+UTCCFtEaQft2ru0Tv+30Un82gG1uEzcrMc/8F3lefedcOTrelPQx8xUD8cME9dj99b5oZWfM60b36/xdswhYF7pygskPtXCS28j81xWKHGNhr5s8xL91cbKOovDzdJYdfVIILZnL1sjdbtN8zW4mULOYHwO4ur "

    private var vuforia: VuforiaLocalizer? = null

    private var tfod: TFObjectDetector? = null

    var position: ObjectPosition = ObjectPosition.UNKNOWN

    val detect: AtomicCommand
        get() = DetectCommand()

    class DetectCommand : AtomicCommand() {
        override val _isDone: Boolean
            get() = TODO()

        override fun start() {

            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia()
            initTfod()

            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             */
            if (tfod != null) {
                tfod!!.activate()

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 16/9).
                tfod!!.setZoom(2.5, 16.0 / 9.0)
            }
        }

        override fun execute() {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                val updatedRecognitions = tfod!!.updatedRecognitions
                val cameraWidth = 1024 // CHANGE TO CAMERA DIMENSIONS
                if (updatedRecognitions != null) {
                    val markerRecognitions = mutableListOf<Recognition>()
                    for(recognition in updatedRecognitions) {
                        if(recognition.label == "Marker") {
                            markerRecognitions.add(recognition)
                        }
                    }
                    if(markerRecognitions.size > 1) {
                        var left = false
                        var middle = false
                        var right = false
                        for(recognition in markerRecognitions) {
                            var recognitionCenter = (recognition.left + recognition.right)/2
                            if(recognitionCenter < cameraWidth/3) {
                                left = true
                            }
                            else if(recognitionCenter > cameraWidth/3 && recognitionCenter < (cameraWidth/3)*2) {
                                middle = true
                            }
                            else if(recognitionCenter > (cameraWidth/3)*2) {
                                right = true
                            }
                        }
                        position = if(left && middle) {
                            ObjectPosition.RIGHT
                        } else if(left && right) {
                            ObjectPosition.MIDDLE
                        } else if(middle && right) {
                            ObjectPosition.LEFT
                        } else {
                            ObjectPosition.INVALID
                        }
                    }
                }
            }
        }

        override fun done(interrupted: Boolean) {

        }

        private fun initVuforia() {
            /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
            val parameters = VuforiaLocalizer.Parameters()
            parameters.vuforiaLicenseKey = ObjectDetection.VUFORIA_KEY
            parameters.cameraName = hardwareMap.get<WebcamName>(WebcamName::class.java, "Webcam 1")

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters)

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }

        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        private fun initTfod() {
            val tfodMonitorViewId: Int = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()
            )
            val tfodParameters = TFObjectDetector.Parameters(tfodMonitorViewId)
            tfodParameters.minResultConfidence = 0.8f
            tfodParameters.isModelTensorFlow2 = true
            tfodParameters.inputSize = 320
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
            tfod!!.loadModelFromAsset(
                ObjectDetection.TFOD_MODEL_ASSET,
                *ObjectDetection.LABELS
            )
        }
    }
}
