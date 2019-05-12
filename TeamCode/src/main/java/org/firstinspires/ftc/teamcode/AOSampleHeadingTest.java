package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigationWebcam;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//This Class is a Test to use TF to get a Heading and then Sample based on feedback from the IMU
//@Disabled
@Autonomous(name = "AOSampleHeadingTest", group = "AutoOp")
public class AOSampleHeadingTest extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    private static final String VUFORIA_KEY = "Adt99vT/////AAABmQfaL1Gc6k6YpSV4p0gyJUg3w2FZlS8RqVrXnweJXsLcl6JrGb5Age+Cv4I9IS+9XG2ZMhWR19WkeOkWkrTXMzKjblOZGI0FC/WUj9CXpGB7wTS8qQuNHut0NT3aZzPjx3aNnjfUCmBvwrCcVHgvLBLU460n9TE9Yug17HApyE+ix9xcJ2J5QtVejR9PNm8SNmpFAEyGmSasukJHF0Em7cNrHAsR5MxPSCBAnQA3B2pL5onCRotpWAu0rcOCYfXWrHeVtYAEHzm3GdFRj73cQ3eGFgOCHJSyMC0AhHH0M8cFlltdKc0f08FHioKrXu8OpvLGCTtYMSZ6Jq2we4+keaVEPBZY6U4YBffLK7jQnP1p";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private TFObjectDetector tfod;

    AutoOpRobot robot = new AutoOpRobot();
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        /*
         * Retrieve the camera we are to use.
         */
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();
        robot.setRobottelemetry(telemetry);
        robot.start();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            //Deploy
            //robot.lower();
            robot.callibrateGyro();

//            while (opModeIsActive()) {
                if (tfod != null) {
                    //Scan for Targets
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    MineralDetected detected = new MineralDetected();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        //**************************************************************************************************************************
                        //***************** Note: Camera mounted left on the Robot, only the left 2 minerals might be in View
                        if (updatedRecognitions.size() >= 2){
                            //If both are Silver, then Gold must be out of view to the Right
                            int silverfound = 0;
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            double targetHeading = 0;
                            boolean targetfound = false;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    telemetry.addData("Gold Mineral Detected: ","Sample Gold detected "  +goldMineralX);
                                    targetHeading = recognition.estimateAngleToObject(AngleUnit.DEGREES);

                                    detected.targetHeading = targetHeading;
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                    telemetry.addData("Silver Mineral Detected: ","Sample 1");
                                    silverfound = silverfound + 1 ;
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                    telemetry.addData("Silver Mineral Detected: ","Sample 2");
                                    silverfound = silverfound + 1 ;
                                }
                            }
                            //Additional angle needs to be added for each option to only target Gold
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left: " + goldMineralX);
                                    //Asume move will block and complete and TF interupted
                                    //moveLeft(targetHeading);
                                    telemetry.addData("Estimate Horizontal Angle to Object:", "Angle: " + targetHeading);
                                    //telemetry.addData("Estimate Distance to Object:", "Distance: " + recognition.);
                                    //robot.imuTurn(1,targetHeading,3);
                                    if(!targetfound) {
                                        targetfound=true;
                                        telemetry.addData("Target found Object:", "Angle: " + robot.getAngle());

                                        detected.position = MineralPosition.LEFT;

//                                        robot.rotate(-1 * (int) Math.round(targetHeading), 0.5);
                                    }
                                    //Sample
                                    //Drop beacon
                                    //Move to crater
                                    //Stop
                                }
                                else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
                                {
                                    telemetry.addData("Gold Mineral Position", "Right: " + goldMineralX);
                                    detected.position = MineralPosition.RIGHT;
                                }
                                else
                                {
                                    telemetry.addData("Gold Mineral Position", "Center: " + goldMineralX);
                                    detected.position = MineralPosition.CENTER;
                                }
                            }
                            if (goldMineralX == -1 && silverfound ==2){
                                telemetry.addData("Gold Mineral Position", "Right: " + goldMineralX +" Silver: " + silverfound) ;
                            }
                        }
                        telemetry.update();
                        if (updatedRecognitions.size() > 2) {

                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            double targetHeading = 0;
                            boolean targetfound = false;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    telemetry.addData("Gold Mineral Detected: ","Sample Gold detected "  +goldMineralX);
                                    targetHeading = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                                    detected.targetHeading = targetHeading;

                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                    telemetry.addData("Silver Mineral Detected: ","Sample 1");
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                    telemetry.addData("Silver Mineral Detected: ","Sample 2");
                                }
                            }
                            //Additional angle needs to be added for each option to only target Gold
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left: " + goldMineralX);
                                    //Asume move will block and complete and TF interupted
                                    //moveLeft(targetHeading);
                                    telemetry.addData("Estimate Horizontal Angle to Object:", "Angle: " + targetHeading);
                                    //telemetry.addData("Estimate Distance to Object:", "Distance: " + recognition.);
                                    //robot.imuTurn(1,targetHeading,3);
                                    if(!targetfound) {

                                        telemetry.addData("Target found Object:", "Angle: " + robot.getAngle());
                                        targetfound=true;
                                        //robot.rotate(-1 * (int) Math.round(targetHeading), 0.5);
                                        detected.position = MineralPosition.LEFT;
                                    }
                                    //Sample
                                    //Drop beacon
                                    //Move to crater
                                    //Stop
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right: " + goldMineralX);
                                    detected.position = MineralPosition.RIGHT;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center: " + goldMineralX);
                                    detected.position = MineralPosition.CENTER;
                                }
                            }
                        }
                        telemetry.update();
                    }

                    if(detected.targetHeading >= 0){
                        telemetry.addData("Moving!", "H,P:(" + detected.targetHeading + "),(" + detected.position + ")");
                        robot.rotate(-1 * (int) Math.round(detected.targetHeading), 0.5);
                        robot.SampleMineral(); //Lower Lift and start Collector
                        robot.imuDriveStraight(0.3,250,3);
                    }

                    //Now Go and drop Beacon or Drive to Crator, See OpModes
                }
//            }
        }
        //Shutdown to release resources
        if (tfod != null) {
            tfod.shutdown();
        }


    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.useExtendedTracking = true;

        /**
         * We also indicate which camera on the RC we wish to use. For pedagogical purposes,
         * we use the same logic as in {@link ConceptVuforiaNavigationWebcam}.
         */
        parameters.cameraName = robot.getWebcamName();
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.minimumConfidence=0.9;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        //Object Tracker is by default on
        //tfodParameters.useObjectTracker = true;
    }
}

enum MineralPosition {
    LEFT,CENTER,RIGHT
}
class MineralDetected{

    double targetHeading = 0;

    double distance;

    MineralPosition position;


}