package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigationWebcam;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

//This OpMode Sample across the Crator, Drop the Beacon and Park again at the Crator from the side
@Autonomous(name = "AOSampleAtCraterDrpBconPark", group = "AutoOp")
public class AOSampleAtCraterDropBeaconParkCrator extends LinearOpMode {
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
    WebcamName webcamName;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private TFObjectDetector tfod;

    AutoOpRobot robot = new AutoOpRobot();
    @Override
    public void runOpMode() throws InterruptedException {

        try{
            robot.init(hardwareMap);

            telemetry.addData(">", "Wait for initialization");
            telemetry.update();
            /*
             * Retrieve the camera we are to use.
             */
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            //robot.initCamera();

            initVuforia();
            //sleep(1000);
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

            MineralDetected detected = new MineralDetected();

            telemetry.addData(">", "Press Play to start");
            telemetry.update();

            waitForStart();
            robot.setRobottelemetry(telemetry);
            robot.start();
            //unhitchRobot();

            if (opModeIsActive()) {
                /** Activate Tensor Flow Object Detection. */
                boolean targetfound = false;
                if (tfod != null) {
                    tfod.activate();
                }
                boolean sampledetected =false;

                while (!sampledetected) {

                    if (tfod != null && !sampledetected) {
                        //Scan for Targets
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() >= 2) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                double targetHeading = 0;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                        telemetry.addData("Gold Mineral Detected: ","Left: "  +goldMineralX);
                                        telemetry.addData("Estimate Horizontal Angle to Object:", "Angle: " + formatDegrees(recognition.estimateAngleToObject(AngleUnit.DEGREES)));
                                        //telemetry.addData("Estimate Distance to Object:", "Distance: " + recognition.);
                                        targetHeading = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                                        telemetry.addData("Gold(1)","Pos:(L "  + recognition.getLeft() + ") B:(" + recognition.getBottom() + ")");

                                        silverMineral2X = 600;
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                        telemetry.addData("Silver Mineral Detected: ","Left 1: " + silverMineral1X);
                                        telemetry.addData("Sample(1)","Pos:(L "  + recognition.getLeft() + ") B:(" + recognition.getBottom() + ")");
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                        telemetry.addData("Silver Mineral Detected: ","Left 2:" +silverMineral2X);
                                        telemetry.addData("Sample(2)","Pos:(L "  + recognition.getLeft() + ") B:(" + recognition.getBottom() + ")");
                                    }
                                }
                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Left: " + goldMineralX);
                                        //Asume move will block and complete and TF interupted
                                        detected.targetHeading = targetHeading;
                                        detected.position = MineralPosition.LEFT;
                                        sampledetected = true;
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Right: " + goldMineralX);
                                        detected.targetHeading = targetHeading;
                                        detected.position = MineralPosition.RIGHT;
                                        sampledetected = true;
                                    } else {
                                        telemetry.addData("Gold Mineral Position", "Center: " + goldMineralX);
                                        detected.targetHeading = targetHeading;
                                        detected.position = MineralPosition.CENTER;
                                        sampledetected = true;
                                    }
                                }
                                //We can't detect the far right because of the camera position so we concentrate on what we can see
                                //If both samples are silver we assume the 3rd (Far right which we can't see) is the Gold mineral
                                //If either of the 2 are Gold or Silver, we just test which are on the Left
                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X == -1) {
                                    if (goldMineralX < silverMineral1X) {
                                        detected.targetHeading = targetHeading;
                                        detected.position = MineralPosition.LEFT;
                                        telemetry.addData("Gold Mineral Position", "Left: " + goldMineralX);
                                        telemetry.addData("Gold Heading", "Degrees: " + targetHeading);
                                        //Asume move will block and complete and TF interupted
                                        sampledetected = true;
                                    } else if (goldMineralX > silverMineral1X) {
                                        detected.targetHeading = targetHeading;
                                        detected.position = MineralPosition.RIGHT;
                                        telemetry.addData("Gold Mineral Position", "Center: " + goldMineralX);
                                        telemetry.addData("Gold Heading", "Degrees: " + targetHeading);
                                        //moveRight(targetHeading);
                                        sampledetected = true;
                                    }
                                }
                                //Calculated guess that Gold is far right
                                if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                    detected.targetHeading = -16;
                                    detected.position = MineralPosition.RIGHT;
                                    telemetry.addData("Gold Mineral Position", "Right: " + goldMineralX);
                                    telemetry.addData("Gold Heading", "Degrees: " + targetHeading);
                                    sampledetected = true;
                                }
                            }
                            telemetry.update();
                        }
                    }

                    if(sampledetected){
                        //Sample and add Path and Park in Crator
                        telemetry.addData("Moving to Mineral!", "Hdng,Pos:(" + detected.targetHeading + "),(" + detected.position + ")");
                        //From the Lander
                        //1. Rotate towards mineral
                        //2. Setup collector and drive towards sample
                        //3. Turn to avoid Silver backup to Save Location
                        //4. Lift Arm to fit over crator
                        //5. Drive and Park from a central point for all collection
                        switch (detected.position)
                        {
                            case LEFT:
                                setupCollectorliftarm();
                                robot.rotate(-1 * (int) Math.round(detected.targetHeading), 0.6);
                                robot.encoderDriveStraight(0.6, 500, 4);
                                robot.sampleMineral();
                                //Move a bit forward to make sure if we drop the mineral its completely moved
                                robot.encoderDriveStraight(0.6, 50, 4);
                                //Lift the arm up higher than the sides because we might over reach
                                robot.encoderMoveLift(1200, 1, 3);
                                //Reverse a bit
                                robot.encoderDriveStraight(0.6, -50, 4);
                                //Turn left towards Depot
                                robot.rotate(30,0.6);
                                //Drive to Depot
                                robot.encoderDriveStraight(0.7,200,4);
                                //Turn to Drop
                                robot.rotate(90,0.8);
                                //Drop beacon
                                robot.dropBeacon(2);
                                //Turn Right againrive
                                robot.rotate(90,0.8);
                                //Drive to crator
                                robot.encoderDriveStraight(1,300,5);
                                //We might need to extend and lift, but we will Test first
                                break;
                            case CENTER:
                                setupCollectorliftarm();
                                robot.encoderDriveStraight(0.6, 500, 4);
                                robot.sampleMineral();
                                //Move a bit forward to make sure if we drop the mineral its completely moved
                                robot.encoderDriveStraight(0.6, 50, 4);
                                //Lift the arm up to lear sides of field
                                robot.encoderMoveLift(1200, 1, 3);
                                //Distances and headings need to be Tested and validated
                                //Reverse a bit
                                robot.encoderDriveStraight(0.6, -50, 4);
                                //Turn left towards Depot
                                robot.rotate(35,0.6);
                                //Drive to Depot
                                robot.encoderDriveStraight(0.7,200,4);
                                //Turn to Drop
                                robot.rotate(90,0.8);
                                //Drop beacon
                                robot.dropBeacon(2);
                                //Turn Right againrive
                                robot.rotate(90,0.8);
                                //Drive to crator
                                robot.encoderDriveStraight(1,300,5);
                                //We might need to extend and lift, but we will Test first
                                break;
                            case RIGHT:
                                setupCollectorliftarm();
                                robot.rotate(-16, 0.6);
                                robot.encoderDriveStraight(0.6, 500, 4);
                                robot.sampleMineral();
                                //Move a bit forward to make sure if we drop the mineral its completely moved
                                robot.encoderDriveStraight(0.6, 50, 4);
                                //Lift the arm up to lear sides of field
                                robot.encoderMoveLift(1200, 1, 3);
                                //Distances and headings need to be Tested and validated
                                //Reverse a bit
                                robot.encoderDriveStraight(0.6, -50, 4);
                                //Turn left towards Depot
                                robot.rotate(40,0.6);
                                //Drive to Depot
                                robot.encoderDriveStraight(0.7,200,4);
                                //Turn to Drop
                                robot.rotate(90,0.8);
                                //Drop beacon
                                robot.dropBeacon(2);
                                //Turn Right againrive
                                robot.rotate(90,0.8);
                                //Drive to crator
                                robot.encoderDriveStraight(1,300,5);
                                //We might need to extend and lift, but we will Test first
                                break;
                            default://Move to the center
                                setupCollectorliftarm();
                                robot.encoderDriveStraight(0.6, 500, 4);
                                robot.sampleMineral();
                                //Move a bit forward to make sure if we drop the mineral its completely moved
                                robot.encoderDriveStraight(0.6, 50, 4);
                                robot.encoderMoveLift(1200, 1, 3);
                                //Distances and headings need to be Tested and validated
                                //Reverse a bit
                                robot.encoderDriveStraight(0.6, -50, 4);
                                //Turn left towards Depot
                                robot.rotate(35,0.6);
                                //Drive to Depot
                                robot.encoderDriveStraight(0.7,200,4);
                                //Turn to Drop
                                robot.rotate(90,0.8);
                                //Drop beacon
                                robot.dropBeacon(2);
                                //Turn Right againrive
                                robot.rotate(90,0.8);
                                //Drive to crator
                                robot.encoderDriveStraight(1,300,5);
                                //We might need to extend and lift, but we will Test first
                                break;
                        }
                    }
                }
            }
            //Shutdown to release resources
            if (tfod != null) {
                tfod.shutdown();
            }
        }
        catch (Exception ex){
            telemetry.addData("Exception ", ex.getMessage());
            if (tfod != null) {
                tfod.shutdown();
            }
        }

    }

    //Drive from a predefined position, then drop Beacon and Park at the Crator
    private void DropAndPark(int initialturn, int backupPathLength){
        //Reverse a bit
        robot.encoderDriveStraight(0.6, -50, 4);
        //Turn left towards Depot
        robot.rotate(35,0.6);
        //Drive to Depot
        robot.encoderDriveStraight(0.7,200,4);
        //Turn to Drop
        robot.rotate(90,0.8);
        //Drop beacon
        robot.dropBeacon(2);
        //Turn Right againrive
        robot.rotate(90,0.8);
        //Drive to crator
        robot.encoderDriveStraight(1,300,5);
        //We might need to extend and lift, but we will Test first
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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
        parameters.cameraName = webcamName;
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

    private void unhitchRobot() {
        //Take xtra care of encoder settings, like RUN_TO_POSITION, if you need to keep track of positions in every step
        robot.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Lift and extend
        telemetry.addLine("Start Lowering the Robot");
        telemetry.update();
        robot.lower(); // See the changes to keep track of position
        telemetry.update();
    }

    private void setupCollectorliftarm(){

        robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorLift.setDirection(DcMotorSimple.Direction.REVERSE);

        //Lift and extend
        telemetry.addData("Lift Current Position",  "Target :%7d", robot.motorLift.getCurrentPosition());
        //Negative is Lift -700 Extender +3700
        //robot.encoderMoveLift(500, 1, 3);
        robot.encoderExtender(1700,0.8,3);
        telemetry.update();
    }
}

