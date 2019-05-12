package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class AutoOpRobot extends LinearOpMode {

    public DcMotor motorLeftFront;
    public DcMotor motorLeftRear;
    public DcMotor motorRightFront;
    public DcMotor motorRightRear;
    public DcMotor motorCollect;
    public DcMotor motorLift;
    public DcMotor motorExtend;
    public Servo hitchServo;
    public Servo dropBeaconServo;

    boolean lower = false;
    boolean pit = false;

    HardwareMap masterConfig = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40 ;     // This is < 1.0 if geared UP 40:1 reduce to 160 rpm
    static final double     WHEEL_DIAMETER_MM   = 100 ;     // For figuring circumference
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);

    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.65;
    static final double     STRAFE_SPEED            = 0.5;

    private Telemetry robottelemetry;

    boolean lastResetState = false;
    boolean curResetState  = false;

    //private WebcamName webcamName;

    private BNO055IMU imu;

    private Orientation angles;
    private Acceleration gravity;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public AutoOpRobot() {

    }

    @Override
    public void runOpMode() {

    }

    public void init(HardwareMap amasterConfig) {

        masterConfig = amasterConfig;

        hitchServo = masterConfig.servo.get("hitchServo");
        motorCollect = masterConfig.get(DcMotor.class, "motorCollect");
        motorExtend = masterConfig.get(DcMotor.class, "motorExtend");
        motorLift = masterConfig.get(DcMotor.class, "motorLift");
        motorLeftFront = masterConfig.get(DcMotor.class, "motorLeftFront");
        motorLeftRear = masterConfig.get(DcMotor.class, "motorLeftRear");
        motorRightFront = masterConfig.get(DcMotor.class, "motorRightFront");
        motorRightRear = masterConfig.get(DcMotor.class, "motorRightRear");

        //Set the Direction of the Motors on the right side AFTER THOROUGH INSPECTION of electrics
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorRightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorCollect = masterConfig.get(DcMotor.class, "motorCollect");

        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This will reset the Position to Zero !!!!!!
        motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dropBeaconServo = masterConfig.servo.get("DropBeaconServo");

        dropBeaconServo.setPosition(0.5);

        //webcamName = masterConfig.get(WebcamName.class, "Webcam 1");

                //robottelemetry.addData("Motor Config added:" ,"Value: ");

        imu = masterConfig.get(BNO055IMU.class, "imu");

        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";

        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(150);
            idle();
        }
        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        //modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    }

//    public void initCamera(){
//        webcamName = masterConfig.get(WebcamName.class, "Webcam 1");
//    }

    public Telemetry getRobottelemetry() {
        return robottelemetry;
    }

    public void setRobottelemetry(Telemetry robottelemetry) {
        this.robottelemetry = robottelemetry;
    }

    public void callibrateGyro(){
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        robottelemetry.addData("Auto OP imu calib status", imu.getCalibrationStatus().toString());
        robottelemetry.update();
//
//        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
//        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
//        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
//        // A similar approach will work for the Gyroscope interface, if that's all you need.
//
//        // Start calibrating the gyro. This takes a few seconds and is worth performing
//        // during the initialization phase at the start of each opMode.
//        robottelemetry.log().add("Gyro Calibrating. Do Not Move!");
//        modernRoboticsI2cGyro.calibrate();
//
//        // Wait until the gyro calibration is complete
//        timer.reset();
//        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
//            robottelemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
//            robottelemetry.update();
//            sleep(50);
//        }
//
//        robottelemetry.log().clear(); robottelemetry.log().add("Gyro Calibrated. Press Start.");
//        robottelemetry.clear(); robottelemetry.update();
    }

    public void resetGyro(){
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        sendTelemetry();


//        // Before the OpMode is started we reset the Gyro
//        curResetState = true;
//        if (curResetState && !lastResetState) {
//            modernRoboticsI2cGyro.resetZAxisIntegrator();
//        }
    }

    void sendTelemetry()
    {
        robottelemetry.addData("AO Status", imu.getSystemStatus().toString());
        robottelemetry.addData("AO Calib", imu.getCalibrationStatus().toString());
        robottelemetry.addData("AO Heading", formatAngle(angles.angleUnit, angles.firstAngle));
        robottelemetry.addData("AO Roll", formatAngle(angles.angleUnit, angles.secondAngle));
        robottelemetry.addData("AO Pitch", formatAngle(angles.angleUnit, angles.thirdAngle));

        robottelemetry.addData("AO Grav", gravity.toString());
        robottelemetry.update();
    }

    /**
     * @param speed where 1 is full speed, 0.5 is halve speed
     * @param distanceMM the disctance in mm the robot will move
     * @param timeoutS A timeout savety factor
     */
    public void encoderDriveStraight(double speed, double distanceMM, double timeoutS) {
        int newLeftFrontTarget;
        int newLeftRearTarget;
        int newRightFrontTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //We use Tank Drive in all 4 wheels, so make sure we sent the correct signals to both Left and Right wheels
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = motorLeftFront.getCurrentPosition() + (int)(distanceMM * COUNTS_PER_MM);
            newLeftRearTarget = motorLeftRear.getCurrentPosition() + (int)(distanceMM * COUNTS_PER_MM);
            newRightFrontTarget = motorRightFront.getCurrentPosition() + (int)(distanceMM * COUNTS_PER_MM);
            newRightRearTarget = motorRightRear.getCurrentPosition() + (int)(distanceMM * COUNTS_PER_MM);

            motorLeftFront.setTargetPosition(newLeftFrontTarget);
            motorLeftRear.setTargetPosition(newLeftRearTarget);
            motorRightFront.setTargetPosition(newRightFrontTarget);
            motorRightRear.setTargetPosition(newRightRearTarget);

            enableRunToPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeftFront.setPower(Math.abs(speed));
            motorRightFront.setPower(Math.abs(speed));

            motorLeftRear.setPower(Math.abs(speed));
            motorRightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits, we should monitor ALL 4
            //but assume for noe only 2 or monitor 2
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH (actually all 4)  motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            //Only monitor the Back wheels
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&  (motorLeftRear.isBusy() ||  motorRightRear.isBusy()))
            {
                //Get the GPS data, the differential should be used to re-align the Robot

                lastResetState = curResetState;

//                // The raw() methods report the angular rate of change about each of the
//                // three axes directly as reported by the underlying sensor IC.
//                int rawX = modernRoboticsI2cGyro.rawX();
//                int rawY = modernRoboticsI2cGyro.rawY();
//                int rawZ = modernRoboticsI2cGyro.rawZ();
//                int heading = modernRoboticsI2cGyro.getHeading();
//                int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
//
//                // Read dimensionalized data from the gyro. This gyro can report angular velocities
//                // about all three axes. Additionally, it internally integrates the Z axis to
//                // be able to report an absolute angular Z orientation.
//                AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
//                float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//
//                // Read administrative information from the gyro
//                int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
//                int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();
//
//                robottelemetry.addLine()
//                        .addData("dx", formatRate(rates.xRotationRate))
//                        .addData("dy", formatRate(rates.yRotationRate))
//                        .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
//                robottelemetry.addData("angle", "%s deg", formatFloat(zAngle));
//                robottelemetry.addData("heading", "%3d deg", heading);
//                robottelemetry.addData("integrated Z", "%3d", integratedZ);
//                robottelemetry.addLine()
//                        .addData("rawX", formatRaw(rawX))
//                        .addData("rawY", formatRaw(rawY))
//                        .addData("rawZ", formatRaw(rawZ));
//                robottelemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
//                robottelemetry.update();

                // Display it for the Debugging.
                robottelemetry.addData("Path1",  "Running to Target LF,LR, RF, RR %7d :%7d :%7d :%7d", newLeftFrontTarget,  newLeftRearTarget, newRightFrontTarget,newRightRearTarget);
                robottelemetry.update();
            }

            // Stop all motion after Path is completed;
            motorLeftFront.setPower(0);
            motorLeftRear.setPower(0);

            motorRightFront.setPower(0);
            motorRightRear.setPower(0);
            // Turn off RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

//    /**
//     * This Method will incrementaly turn the Robot untill the Giro tells it to stop
//     * @param speed speed for motors
//     * @param heading Degrees the robot must turn relative to last position
//     * @param timeoutS Timeout
//     */
//    public void imuTurn(double speed, double heading, double timeoutS){
//        // Ensure that the opmode is still active
//        // reset the timeout time and start motion.
//        runtime.reset();
//        //Reset imu Global heading to start with a relative zero heading
//        resetAngle();
//
//        if (opModeIsActive()) {
//            //We use Tank Drive in all 4 wheels, so make sure we sent the correct signals to both Left and Right wheels
//            // Determine new target position, and pass to motor controller
//           // keep looping while we are still active, and there is time left, and both motors are running.
//            //We use the IMU here to turn untill heading is reached
//            while (opModeIsActive() && runtime.seconds() < timeoutS && getAngles().firstAngle <= heading)
//            {
//                // Display it for the Debugging.
//                robottelemetry.addData("Heading",  "Running to Target Heading %7d ", formatAngle(angles.angleUnit, angles.firstAngle));
//                robottelemetry.update();
//                if(angles.firstAngle<heading)
//                {
//                    //Turn left
//                    motorLeftFront.setPower(-speed);
//                    motorLeftRear.setPower(-speed);
//
//                    motorRightFront.setPower(speed);
//                    motorRightRear.setPower(speed);
//                }
//                else
//                {
//                    //Turn Right
//                    motorLeftFront.setPower(speed);
//                    motorLeftRear.setPower(speed);
//
//                    motorRightFront.setPower(-speed);
//                    motorRightRear.setPower(-speed);
//                }
//            }
//
//            robottelemetry.addData("Power Reset","Power set to 0");
//            robottelemetry.update();
//            // Stop all motion after Path is completed;
//            motorLeftFront.setPower(0);
//            motorLeftRear.setPower(0);
//
//            motorRightFront.setPower(0);
//            motorRightRear.setPower(0);
//            // Turn off RUN_TO_POSITION
//            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            //Reset Again
//            resetAngle();
//        }
//    }
//    public void driveCollector(boolean collect, double speed){
////        public void Sample(){
////            encoderMoveLift(-2000, 1, 5);
////            encoderExtender(-2600, 1, 5);
////            motorCollect.setPower(0.8);
////            encoderMoveLift(3100, 1, 5);
////            motorCollect.setPower(0);
////        }
//    }

    public void sampleMineral(){
        encoderMoveLift(2000, 1, 5);
        encoderExtender(-260, 1, 5);
        motorCollect.setPower(0.8);
//        encoderMoveLift(3100, 1, 5);
//        motorCollect.setPower(0);
    }

    private void disableEncoders(){
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void enableEncoders(){
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void enableRunToPosition(){
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * @param power Setting to set Robot speed, double from 0 -1
     * @param distanceMM distance in mm to move
     * @param timeoutS Safety Timout
     */
    public void imuDriveStraight(double power, int distanceMM, double timeoutS){

        int newLeftFrontTarget;
        int newLeftRearTarget;
        int newRightFrontTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robottelemetry.addData("1 imu heading", lastAngles.firstAngle);
            robottelemetry.addData("2 correction", correction);
            robottelemetry.update();

            //leftMotor.setPower(power - correction);
            //rightMotor.setPower(power + correction);

            //We use Tank Drive in all 4 wheels, so make sure we sent the correct signals to both Left and Right wheels
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = motorLeftFront.getCurrentPosition() + (int)(distanceMM * COUNTS_PER_MM);
            newLeftRearTarget = motorLeftRear.getCurrentPosition() + (int)(distanceMM * COUNTS_PER_MM);
            newRightFrontTarget = motorRightFront.getCurrentPosition() + (int)(distanceMM * COUNTS_PER_MM);
            newRightRearTarget = motorRightRear.getCurrentPosition() + (int)(distanceMM * COUNTS_PER_MM);

            motorLeftFront.setTargetPosition(newLeftFrontTarget);
            motorLeftRear.setTargetPosition(newLeftRearTarget);
            motorRightFront.setTargetPosition(newRightFrontTarget);
            motorRightRear.setTargetPosition(newRightRearTarget);

            // reset the timeout time and start motion.
            runtime.reset();
            //Left
            motorLeftFront.setPower(Math.abs(power ));
            motorLeftRear.setPower(Math.abs(power));
            //Right
            motorRightFront.setPower(Math.abs(power));
            motorRightRear.setPower(Math.abs(power));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits, we should monitor ALL 4
            //but assume for noe only 2 or monitor 2
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH (actually all 4)  motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            //Only monitor the Back wheels
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&  (motorLeftRear.isBusy() ||  motorRightRear.isBusy()))
            {
                // Use gyro to drive in a straight line.
                correction = checkDirection();

                //Get the IMU data, and apply a correction if need
                motorLeftFront.setPower(Math.abs(power - correction));
                motorLeftRear.setPower(Math.abs(power - correction));

                motorRightFront.setPower(Math.abs(power + correction));
                motorRightRear.setPower(Math.abs(power + correction));
                lastResetState = curResetState;
                robottelemetry.addData("Path1",  "Running to Target LF,LR, RF, RR %7d :%7d :%7d :%7d", newLeftFrontTarget,  newLeftRearTarget, newRightFrontTarget,newRightRearTarget);
                robottelemetry.update();
            }

            // Stop all motion after Path is completed;
            motorLeftFront.setPower(0);
            motorLeftRear.setPower(0);

            motorRightFront.setPower(0);
            motorRightRear.setPower(0);
            // Turn off RUN_TO_POSITION
            enableEncoders();
        }

    }

    public void encoderTurn(double speed, double leftMMdistance, double rightMMdistance, double timeoutS){
        int newLeftFrontTarget;
        int newLeftRearTarget;
        int newRightFrontTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            //We use Tank Drive in all 4 wheels, so make sure we sent the correct signals to both Left and Right wheels
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = motorLeftFront.getCurrentPosition() + (int)(leftMMdistance * COUNTS_PER_MM);
            newLeftRearTarget = motorLeftRear.getCurrentPosition() + (int)(leftMMdistance * COUNTS_PER_MM);
            newRightFrontTarget = motorRightFront.getCurrentPosition() + (int)(rightMMdistance * COUNTS_PER_MM);
            newRightRearTarget = motorRightRear.getCurrentPosition() + (int)(rightMMdistance * COUNTS_PER_MM);

            motorLeftFront.setTargetPosition(newLeftFrontTarget);
            motorLeftRear.setTargetPosition(newLeftRearTarget);
            motorRightFront.setTargetPosition(newRightFrontTarget);
            motorRightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            enableRunToPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeftFront.setPower(Math.abs(speed));
            motorRightFront.setPower(Math.abs(speed));

            motorLeftRear.setPower(Math.abs(speed));
            motorRightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            //Only monitor the Back wheels
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motorLeftRear.isBusy() ||  motorRightRear.isBusy()))
            {

                // Display it for the Debugging.
                robottelemetry.addData("Path1",  "Running to Target LF,LR, RF, RR %7d :%7d :%7d :%7d", newLeftFrontTarget,  newLeftRearTarget, newRightFrontTarget,newRightRearTarget);
                robottelemetry.update();
            }

            robottelemetry.addData("Power Reset","Power set to 0");
            robottelemetry.update();
            // Stop all motion after Path is completed;
            motorLeftFront.setPower(0);
            motorLeftRear.setPower(0);

            motorRightFront.setPower(0);
            motorRightRear.setPower(0);
            // Turn off RUN_TO_POSITION
            enableEncoders();
        }
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        disableEncoders();
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        //leftMotor.setPower(leftPower);
        //rightMotor.setPower(rightPower);
        motorLeftFront.setPower(leftPower);
        motorLeftRear.setPower(leftPower);

        motorRightFront.setPower(rightPower);
        motorRightRear.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        motorLeftFront.setPower(0);
        motorLeftRear.setPower(0);

        motorRightFront.setPower(0);
        motorRightRear.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void lower() {

        while (!lower) {
            //IMPORTANT remove encoder reset etc from lower methods or you will reset poistions every time in the methods and won't be able to keep track
            //as an alternative keep a global variable for positions for each motor

            robottelemetry.addData("Lift Current Position",  "Target :%7d", motorLift.getCurrentPosition());
            encoderMoveLift(10500, 1, 5);
            robottelemetry.update();
            robottelemetry.addData("Extender Position",  "Target :%7d", motorExtend.getCurrentPosition());
            encoderExtender(-2100, 1, 5);
            //Unhtch servo here
            //do {
            hitchServo.setPosition(0);
            //} while (hitchServo.getPosition() != 0);
            sleep(1000);
            //Move lift to horizontal (make sure position is not Reset, this is a differential from current position (9200) relative to where we started at 0
            encoderMoveLift(-7600,1,5);
            robottelemetry.addData("Lift Current Position",  "Target :%7d", motorLift.getCurrentPosition());
            //encoderExtender(700, 1, 5);
            robottelemetry.addData("Extender Position",  "Target :%7d", motorExtend.getCurrentPosition());
            //Lower for drive
            robottelemetry.update();
            lower = true;
            hitchServo.setPosition(1);
//            motorLift.setPower(1);
//            sleep(3500);
//            motorLift.setPower(0);
//            motorExtend.setPower(-1);
//            sleep(700);
//            motorExtend.setPower(0);
//
//            hitchServo.setPower(1);
//            sleep(2000);
//            hitchServo.setPower(0);
        }
        // lower = true;
    }

    //Use the Rev4wheel Telop to get the Values for various positions
    public void encoderMoveLift(int position, double speed,double timeoutS)
    {
        int newLiftget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftget = motorLift.getCurrentPosition() + position;//+ (int)(distanceMM * COUNTS_PER_MM);

            motorLift.setTargetPosition(newLiftget);
            //// Turn On RUN_TO_POSITION
            //motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            motorLift.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits, we should monitor ALL 4
            //but assume for noe only 2 or monitor 2
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH (actually all 4)  motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLift.isBusy() ))
            {

                // Display it for the Debugging.
                robottelemetry.addData("Lift Path",  "Running to Target :%7d", newLiftget);
                robottelemetry.update();
            }

            // Stop all motion after Path is completed;
            motorLift.setPower(0);
            // Turn off RUN_TO_POSITION
            //motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    //Use the Rev4wheel Telop to get the Values for various positions
    public void encoderExtender(int position,double speed, double timeoutS)
    {
        int newExtendget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newExtendget = motorExtend.getCurrentPosition() + position ;//+ (int)(distanceMM * COUNTS_PER_MM);


            motorExtend.setTargetPosition(newExtendget);
            // Turn On RUN_TO_POSITION
//            motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            motorExtend.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits, we should monitor ALL 4
            //but assume for noe only 2 or monitor 2
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH (actually all 4)  motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorExtend.isBusy() ))
            {

                // Display it for the Debugging.
                robottelemetry.addData("Extender Path",  "Running to Target :%7d", position);
                robottelemetry.update();
            }

            // Stop all motion after Path is completed;
            motorExtend.setPower(0);
            // Turn off RUN_TO_POSITION
            //motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void MoveTillEnd() {
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double driveDistance1 = 1200 * COUNTS_PER_MM;
        double startPos1 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance1 + startPos1) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(+1);
            motorRightRear.setPower(+1);
            robottelemetry.addLine("Forward");
        }

        motorLift.setPower(-1);
        sleep(2000);
        motorLift.setPower(0);
    }

    public void stopRobot() {
        motorRightFront.setPower(0);
        motorRightRear.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftRear.setPower(0);
    }

//    public void turnAroundUntilFound() {
//        motorRightFront.setPower(0.2);
//        motorRightRear.setPower(0.2);
//        motorLeftFront.setPower(-0.2);
//        motorLeftRear.setPower(-0.2);
//    }
//
//    public void MoveL() {
//        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double driveDistance = 100 * COUNTS_PER_MM;
//        double startPos = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance + startPos) {
//            motorLeftFront.setPower(+1);
//            motorLeftRear.setPower(-1);
//            motorRightFront.setPower(+1);
//            motorRightRear.setPower(-1);
//            robottelemetry.addLine("Move Left");
//        }
//    }
//
//    public void ResetLeft() {
//        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double driveDistance1 = 100 * COUNTS_PER_MM;
//        double startPos1 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance1 + startPos1) {
//            motorLeftFront.setPower(-1);
//            motorLeftRear.setPower(-1);
//            motorRightFront.setPower(-1);
//            motorRightRear.setPower(-1);
//            robottelemetry.addLine("Repo");
//        }
//
//        double driveDistance2 = 100 * COUNTS_PER_MM;
//        double startPos2 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance2 + startPos2) {
//            motorLeftFront.setPower(-1);
//            motorLeftRear.setPower(+1);
//            motorRightFront.setPower(-1);
//            motorRightRear.setPower(+1);
//            robottelemetry.addLine("Move Left");
//        }
//    }
//
//    public void MoveR() {
//        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double driveDistance = 100 * COUNTS_PER_MM;
//        double startPos = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance + startPos) {
//            motorLeftFront.setPower(-1);
//            motorLeftRear.setPower(+1);
//            motorRightFront.setPower(-1);
//            motorRightRear.setPower(+1);
//            robottelemetry.addLine("Move Right");
//        }
//    }
//
//    public void ResetRight() {
//        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double driveDistance1 = 100 * COUNTS_PER_MM;
//        double startPos1 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance1 + startPos1) {
//            motorLeftFront.setPower(-1);
//            motorLeftRear.setPower(-1);
//            motorRightFront.setPower(-1);
//            motorRightRear.setPower(-1);
//            robottelemetry.addLine("Repo");
//        }
//
//        double driveDistance2 = 100 * COUNTS_PER_MM;
//        double startPos2 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance2 + startPos2) {
//            motorLeftFront.setPower(+1);
//            motorLeftRear.setPower(-1);
//            motorRightFront.setPower(+1);
//            motorRightRear.setPower(-1);
//            robottelemetry.addLine("Move Right");
//        }
//    }

//    public void Sample() {
//        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double driveDistance = 100 * COUNTS_PER_MM;
//        double startPos = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance + startPos) {
//            motorLeftFront.setPower(+1);
//            motorLeftRear.setPower(+1);
//            motorRightFront.setPower(+1);
//            motorRightRear.setPower(+1);
//            robottelemetry.addLine("Sample");
//        }
//    }

//    public void ResetMid() {
//        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double driveDistance = 100 * COUNTS_PER_MM;
//        double startPos = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance + startPos) {
//            motorLeftFront.setPower(-1);
//            motorLeftRear.setPower(-1);
//            motorRightFront.setPower(-1);
//            motorRightRear.setPower(1);
//            robottelemetry.addLine("Reset");
//        }
//    }
//
//    public void MoveFromCrator() {
//        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        double driveDistance1 = 1400 * COUNTS_PER_MM;
//        double startPos1 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance1 + startPos1) {
//            motorLeftFront.setPower(+1);
//            motorLeftRear.setPower(+1);
//            motorRightFront.setPower(+1);
//            motorRightRear.setPower(+1);
//            robottelemetry.addLine("Forward");
//        }
//
//        double driveDistance2 = 100 * COUNTS_PER_MM;
//        double startPos2 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance2 + startPos2) {
//            motorLeftFront.setPower(-1);
//            motorLeftRear.setPower(-1);
//            motorRightFront.setPower(+1);
//            motorRightRear.setPower(+1);
//            robottelemetry.addLine("Turn");
//        }
//
//        double driveDistance3 = 1200 * COUNTS_PER_MM;
//        double startPos3 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance3 + startPos3) {
//            motorLeftFront.setPower(+1);
//            motorLeftRear.setPower(+1);
//            motorRightFront.setPower(+1);
//            motorRightRear.setPower(+1);
//            robottelemetry.addLine("Forward");
//        }
//
//        double driveDistance4 = 100 * COUNTS_PER_MM;
//        double startPos4 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance4 + startPos4) {
//            motorLeftFront.setPower(+1);
//            motorLeftRear.setPower(+1);
//            motorRightFront.setPower(-1);
//            motorRightRear.setPower(-1);
//            robottelemetry.addLine("Turn");
//        }
//
//        DropBeacon();
//
//        double driveDistance5 = 1880 * COUNTS_PER_MM;
//        double startPos5 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance5 + startPos5) {
//            motorLeftFront.setPower(+1);
//            motorLeftRear.setPower(+1);
//            motorRightFront.setPower(+1);
//            motorRightRear.setPower(+1);
//            robottelemetry.addLine("Turn");
//        }
//    }
//
//    public void MoveFromCorner() {
//        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        double driveDistance3 = 1400 * COUNTS_PER_MM;
//        double startPos3 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance3 + startPos3) {
//            motorLeftFront.setPower(+1);
//            motorLeftRear.setPower(+1);
//            motorRightFront.setPower(+1);
//            motorRightRear.setPower(+1);
//            robottelemetry.addLine("Forward");
//        }
//
//        double driveDistance4 = 100 * COUNTS_PER_MM;
//        double startPos4 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance4 + startPos4) {
//            motorLeftFront.setPower(+1);
//            motorLeftRear.setPower(+1);
//            motorRightFront.setPower(-1);
//            motorRightRear.setPower(-1);
//            robottelemetry.addLine("Turn");
//        }
//
//        DropBeacon();
//
//        double driveDistance5 = 1880 * COUNTS_PER_MM;
//        double startPos5 = motorRightFront.getCurrentPosition();
//        while (motorRightFront.getCurrentPosition() < driveDistance5 + startPos5) {
//            motorLeftFront.setPower(+1);
//            motorLeftRear.setPower(+1);
//            motorRightFront.setPower(+1);
//            motorRightRear.setPower(+1);
//            robottelemetry.addLine("Turn");
//        }
//    }
//
//    public void DropBeacon() {
//        dropBeaconServo.setPosition(0.90);
//        robottelemetry.addData("Drop the Beacon", dropBeaconServo.getPosition());
//
//        dropBeaconServo.setPosition(0.5);
//        robottelemetry.addData("Drop the Beacon", dropBeaconServo.getPosition());
//    }

    String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        robottelemetry.addData("Heading R-", formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));
        robottelemetry.addData("Roll L+", formatAngle(lastAngles.angleUnit, lastAngles.secondAngle));
        robottelemetry.addData("Pitch U-", formatAngle(lastAngles.angleUnit, lastAngles.thirdAngle));
        robottelemetry.addData("Global Heading", formatAngle(lastAngles.angleUnit, globalAngle));

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public void setImu(BNO055IMU imu) {
        this.imu = imu;
    }

    public Orientation getAngles() {
        return angles;
    }

    public void setAngles(Orientation angles) {
        this.angles = angles;
    }

    public Acceleration getGravity() {
        return gravity;
    }

    public void setGravity(Acceleration gravity) {
        this.gravity = gravity;
    }

//    public WebcamName getWebcamName() {
//        return webcamName;
//    }
//
//    public void setWebcamName(WebcamName webcamName) {
//        this.webcamName = webcamName;
//    }
}