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
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutoOpRobot extends LinearOpMode {

    public DcMotor motorLeftFront;
    public DcMotor motorLeftRear;
    public DcMotor motorRightFront;
    public DcMotor motorRightRear;
    public DcMotor motorCollect;
    public DcMotor motorLift;
    public DcMotor motorExtend;
    public CRServo hitchServo;
    public Servo dropBeaconServo;

    private Telemetry robottelemetry;

    boolean lower = false;

    HardwareMap masterConfig = null;
    private ElapsedTime period = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 40;     // This is < 1.0 if geared UP 40:1 reduce to 160 rpm
    static final double WHEEL_DIAMETER_MM = 100;     // For figuring circumference
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);

//    IntegratingGyroscope gyro;
//    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    boolean lastResetState = false;
    boolean curResetState  = false;

    private BNO055IMU imu;

    private Orientation angles;
    private Acceleration gravity;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    public AutoOpRobot() {

    }

    @Override
    public void runOpMode() {

    }

    public void init(HardwareMap amasterConfig) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        setImu(masterConfig.get(BNO055IMU.class, "imu"));
        getImu().initialize(parameters);

        masterConfig = amasterConfig;

        hitchServo = masterConfig.crservo.get("hitchServo");
        motorExtend = masterConfig.get(DcMotor.class, "motorExtend");
        motorLift = masterConfig.get(DcMotor.class, "motorLift");
        motorLeftFront = masterConfig.get(DcMotor.class, "motorLeftFront");
        motorLeftRear = masterConfig.get(DcMotor.class, "motorLeftRear");
        motorRightFront = masterConfig.get(DcMotor.class, "motorRightFront");
        motorRightRear = masterConfig.get(DcMotor.class, "motorRightRear");

        //Set the Direction of the Motors on the right side AFTER THOROUGH INSPECTION of electrics
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
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

        dropBeaconServo = masterConfig.servo.get("DropBeaconServo");
        dropBeaconServo.setPosition(0.5);

        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        //modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    }

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
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
//
//        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
//        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
//        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
//        // A similar approach will work for the Gyroscope interface, if that's all you need.
//
//        // Start calibrating the gyro. This takes a few seconds and is worth performing
//        // during the initialization phase at the start of each opMode.
//        telemetry.log().add("Gyro Calibrating. Do Not Move!");
//        modernRoboticsI2cGyro.calibrate();
//
//        // Wait until the gyro calibration is complete
//        timer.reset();
//        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
//            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
//            telemetry.update();
//            sleep(50);
//        }
//
//        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
//        telemetry.clear(); telemetry.update();
    }

    public void resetGyro(){
//        // Before the OpMode is started we reset the Gyro
//        curResetState = true;
//        if (curResetState && !lastResetState) {
//            modernRoboticsI2cGyro.resetZAxisIntegrator();
//        }
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

            // reset the timeout time and start motion.
            timer.reset();
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
            while (opModeIsActive() && (timer.seconds() < timeoutS) &&  (motorLeftRear.isBusy() ||  motorRightRear.isBusy()))
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

    /**
     * This Method will incrementaly turn the Robot untill the Giro tells it to stop
     * @param speed speed for motors
     * @param heading Degrees the robot must turn relative to last position
     * @param timeoutS Timeout
     */
    public void encoderTurnHeading(double speed, double heading, double timeoutS){


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            //We use Tank Drive in all 4 wheels, so make sure we sent the correct signals to both Left and Right wheels
            // Determine new target position, and pass to motor controller

            // reset the timeout time and start motion.
            timer.reset();
            if(heading>0)//turn right
            {
                motorLeftFront.setPower(Math.abs(speed));
                motorRightFront.setPower(Math.abs(speed));
            }
            else {//Turn Left untill
                motorLeftRear.setPower(Math.abs(speed));
                motorRightRear.setPower(Math.abs(speed));
            }
            // keep looping while we are still active, and there is time left, and both motors are running.
            //Only monitor the Back wheels
            while (opModeIsActive() &&
                    (timer.seconds() < timeoutS) && getAngles().firstAngle < heading)
            {
                angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();

                // Display it for the Debugging.
                robottelemetry.addData("Heading",  "Running to Target Heading %7d ", heading);
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
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timer.reset();
            motorLeftFront.setPower(Math.abs(speed));
            motorRightFront.setPower(Math.abs(speed));

            motorLeftRear.setPower(Math.abs(speed));
            motorRightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            //Only monitor the Back wheels
            while (opModeIsActive() &&
                    (timer.seconds() < timeoutS) &&
                    (motorLeftRear.isBusy() ||  motorRightRear.isBusy()))
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
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void lower() {

        if (lower == false) {

//            motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            double driveDistance2 = 5 * COUNTS_PER_MM;
//            double startPos2 = motorLift.getCurrentPosition();
//            while (motorLift.getCurrentPosition() < driveDistance2 + startPos2) {
//                motorLift.setPower(1);
//            }
//            double driveDistance1 = 50 * COUNTS_PER_MM;
//            double startPos1 = motorExtend.getCurrentPosition();
//            while (motorExtend.getCurrentPosition() < driveDistance1 + startPos1) {
//                motorExtend.setPower(1);
//            }
//            hitchServo.setPower(1);
//            sleep(2000);
//            hitchServo.setPower(0);

            motorLift.setPower(1);
            sleep(3500);
            motorLift.setPower(0);
            motorExtend.setPower(-1);
            sleep(700);
            motorExtend.setPower(0);

            hitchServo.setPower(1);
            sleep(2000);
            hitchServo.setPower(0);
        }
        lower = true;
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
            telemetry.addLine("Forward");
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

    public void turnAroundUntilFound() {
        motorRightFront.setPower(0.2);
        motorRightRear.setPower(0.2);
        motorLeftFront.setPower(-0.2);
        motorLeftRear.setPower(-0.2);
    }

    public void MoveL() {
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double driveDistance = 100 * COUNTS_PER_MM;
        double startPos = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance + startPos) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(-1);
            motorRightFront.setPower(+1);
            motorRightRear.setPower(-1);
            telemetry.addLine("Move Left");
        }
    }

    public void ResetLeft() {
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double driveDistance1 = 100 * COUNTS_PER_MM;
        double startPos1 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance1 + startPos1) {
            motorLeftFront.setPower(-1);
            motorLeftRear.setPower(-1);
            motorRightFront.setPower(-1);
            motorRightRear.setPower(-1);
            telemetry.addLine("Repo");
        }

        double driveDistance2 = 100 * COUNTS_PER_MM;
        double startPos2 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance2 + startPos2) {
            motorLeftFront.setPower(-1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(-1);
            motorRightRear.setPower(+1);
            telemetry.addLine("Move Left");
        }
    }

    public void MoveR() {
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double driveDistance = 100 * COUNTS_PER_MM;
        double startPos = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance + startPos) {
            motorLeftFront.setPower(-1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(-1);
            motorRightRear.setPower(+1);
            telemetry.addLine("Move Right");
        }
    }

    public void ResetRight() {
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double driveDistance1 = 100 * COUNTS_PER_MM;
        double startPos1 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance1 + startPos1) {
            motorLeftFront.setPower(-1);
            motorLeftRear.setPower(-1);
            motorRightFront.setPower(-1);
            motorRightRear.setPower(-1);
            telemetry.addLine("Repo");
        }

        double driveDistance2 = 100 * COUNTS_PER_MM;
        double startPos2 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance2 + startPos2) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(-1);
            motorRightFront.setPower(+1);
            motorRightRear.setPower(-1);
            telemetry.addLine("Move Right");
        }
    }

    public void Sample() {
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double driveDistance = 100 * COUNTS_PER_MM;
        double startPos = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance + startPos) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(+1);
            motorRightRear.setPower(+1);
            telemetry.addLine("Sample");
        }
    }

    public void ResetMid() {
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double driveDistance = 100 * COUNTS_PER_MM;
        double startPos = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance + startPos) {
            motorLeftFront.setPower(-1);
            motorLeftRear.setPower(-1);
            motorRightFront.setPower(-1);
            motorRightRear.setPower(1);
            telemetry.addLine("Reset");
        }
    }

    public void MoveFromCrator() {
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double driveDistance1 = 1400 * COUNTS_PER_MM;
        double startPos1 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance1 + startPos1) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(+1);
            motorRightRear.setPower(+1);
            telemetry.addLine("Forward");
        }

        double driveDistance2 = 100 * COUNTS_PER_MM;
        double startPos2 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance2 + startPos2) {
            motorLeftFront.setPower(-1);
            motorLeftRear.setPower(-1);
            motorRightFront.setPower(+1);
            motorRightRear.setPower(+1);
            telemetry.addLine("Turn");
        }

        double driveDistance3 = 1200 * COUNTS_PER_MM;
        double startPos3 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance3 + startPos3) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(+1);
            motorRightRear.setPower(+1);
            telemetry.addLine("Forward");
        }

        double driveDistance4 = 100 * COUNTS_PER_MM;
        double startPos4 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance4 + startPos4) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(-1);
            motorRightRear.setPower(-1);
            telemetry.addLine("Turn");
        }

        DropBeacon();

        double driveDistance5 = 1880 * COUNTS_PER_MM;
        double startPos5 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance5 + startPos5) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(+1);
            motorRightRear.setPower(+1);
            telemetry.addLine("Turn");
        }
    }

    public void MoveFromCorner() {
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double driveDistance3 = 1400 * COUNTS_PER_MM;
        double startPos3 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance3 + startPos3) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(+1);
            motorRightRear.setPower(+1);
            telemetry.addLine("Forward");
        }

        double driveDistance4 = 100 * COUNTS_PER_MM;
        double startPos4 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance4 + startPos4) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(-1);
            motorRightRear.setPower(-1);
            telemetry.addLine("Turn");
        }

        DropBeacon();

        double driveDistance5 = 1880 * COUNTS_PER_MM;
        double startPos5 = motorRightFront.getCurrentPosition();
        while (motorRightFront.getCurrentPosition() < driveDistance5 + startPos5) {
            motorLeftFront.setPower(+1);
            motorLeftRear.setPower(+1);
            motorRightFront.setPower(+1);
            motorRightRear.setPower(+1);
            telemetry.addLine("Turn");
        }
    }

    public void DropBeacon() {
        dropBeaconServo.setPosition(0.90);
        telemetry.addData("Drop the Beacon", dropBeaconServo.getPosition());

        dropBeaconServo.setPosition(0.5);
        telemetry.addData("Drop the Beacon", dropBeaconServo.getPosition());
    }

    String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
        return String.format("%.3f", rate);
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
}