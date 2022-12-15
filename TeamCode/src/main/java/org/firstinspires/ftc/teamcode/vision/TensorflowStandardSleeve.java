package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.Timeout;

import java.util.List;


public class TensorflowStandardSleeve {

    LinearOpMode opmode;
    public TensorflowStandardSleeve(LinearOpMode opMode){
        opmode = opMode;
    }

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUmiib//////AAABmbrFAeBqbkd/hmTwBoU6jXFUjeYK8xAgeYu6r9ZuLmpgb4tqNl4oIhXkpbBXmCusnhPlxJ3DHEkExTnQKhvCU49Yu2jslI6vaQ+V5F21ZAbbBod6lm9zyBEpkujo7IOq2TdOaJSIdN5wW3zxrHTksfrzBuKZKRsArompruh7jrm/B4W3F/EunA8ymkVoi29W84q81XMwJyonWlS2sd3pebXvLW0YOKmA63QgdmtSpp9XVAccwiH8ND8rk7FXlIIucim1Ig5FmVPLIx88t7doptXh8uiXfHHMqXc1T1MrRvfemYaUqyg7I5lYLNjLhuRmBZO3BM/qoyjPhpMVtGNh6+z3VgaKhP7O6zI07W0mmMfO";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public void init() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        opmode.telemetry.addData(">", "Press Play to start op mode");
        opmode.telemetry.update();
    }

    public TfodSleeve scanStandardSleeve() {
        //start a timer
        if (opmode.opModeIsActive()) {
            Timeout timer = new Timeout (3);

            while (opmode.opModeIsActive() && !timer.expired()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        opmode.telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            //if it sees one, return that
                            if (recognition.getLabel().equals(LABELS[0])){
                                opmode.telemetry.addData("tfodSees","ONE");
                                return TfodSleeve.ONE;
                            }
                            //if it sees two, return that
                            if (recognition.getLabel().equals(LABELS[1])){
                                opmode.telemetry.addData("tfodSees","TWO");
                                return TfodSleeve.TWO;
                            }
                            //if it sees three, return that
                            if (recognition.getLabel().equals(LABELS[2])){
                                opmode.telemetry.addData("tfodSees","THREE");
                                return TfodSleeve.THREE;
                            }

                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            opmode.telemetry.addData(""," ");
                            opmode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            opmode.telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            opmode.telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                        opmode.telemetry.update();
                    }
                }
            }
        }

        opmode.telemetry.log().add("Unable to detect anything, falling back to THREE");
        return TfodSleeve.THREE;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
