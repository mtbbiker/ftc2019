package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class AutoOpTriRobot extends LinearOpMode {

    public DcMotor motorLeftFront;
    public DcMotor motorRightFront;

    HardwareMap masterConfig = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 3.7 ; //40    // This is < 1.0 if geared UP 40:1 reduce to 160 rpm
    static final double     WHEEL_DIAMETER_MM   = 100 ;     // For figuring circumference
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);

    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.65;
    static final double     STRAFE_SPEED            = 0.5;

    private Telemetry robottelemetry;

    private BNO055IMU imu;

    private Orientation angles;
    private Acceleration gravity;

    boolean lastResetState = false;
    boolean curResetState  = false;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void init(HardwareMap amasterConfig) {

        masterConfig = amasterConfig;

        motorLeftFront = masterConfig.get(DcMotor.class, "motorLeftFront");
        motorRightFront = masterConfig.get(DcMotor.class, "motorRightFront");

        //Set the Direction of the Motors on the right side AFTER THOROUGH INSPECTION of electrics
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    private void disableEncoders(){
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void enableEncoders(){
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void enableRunToPosition(){
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     }

    /**
     * @param power Setting to set Robot speed, double from 0 -1
     * @param distanceMM distance in mm to move
     * @param timeoutS Safety Timout
     */
    public void imuDriveStraight(double power, int distanceMM, double timeoutS){

        int newLeftFrontTarget;
        int newRightFrontTarget;


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
            newRightFrontTarget = motorRightFront.getCurrentPosition() + (int)(distanceMM * COUNTS_PER_MM);

            motorLeftFront.setTargetPosition(newLeftFrontTarget);
            motorRightFront.setTargetPosition(newRightFrontTarget);


            // reset the timeout time and start motion.
            runtime.reset();
            //Left
            motorLeftFront.setPower(Math.abs(power ));

            //Right
            motorRightFront.setPower(Math.abs(power));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits, we should monitor ALL 4
            //but assume for noe only 2 or monitor 2
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH (actually all 4)  motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            //Only monitor the Back wheels
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&  (motorLeftFront.isBusy() ||  motorRightFront.isBusy()))
            {
                // Use gyro to drive in a straight line.
                correction = checkDirection();

                //Get the IMU data, and apply a correction if need
                motorLeftFront.setPower(Math.abs(power - correction));

                motorRightFront.setPower(Math.abs(power + correction));
                lastResetState = curResetState;
                robottelemetry.addData("Path-1",  "Running to Target LF, RF, :%7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                robottelemetry.update();
            }

            // Stop all motion after Path is completed;
            motorLeftFront.setPower(0);
            motorRightFront.setPower(0);
            // Turn off RUN_TO_POSITION
            enableEncoders();
        }

    }

    /**
     * @param speed
     * @param leftMMdistance
     * @param rightMMdistance
     * @param timeoutS
     */
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
            newRightFrontTarget = motorRightFront.getCurrentPosition() + (int)(rightMMdistance * COUNTS_PER_MM);

            motorLeftFront.setTargetPosition(newLeftFrontTarget);
            motorRightFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            enableRunToPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeftFront.setPower(Math.abs(speed));
            motorRightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            //Only monitor the Back wheels
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motorLeftFront.isBusy() ||  motorRightFront.isBusy()))
            {

                // Display it for the Debugging.
                robottelemetry.addData("Turn-Path-1",  "Running to Target LF,RF %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                robottelemetry.update();
            }

            robottelemetry.addData("Power Reset","Power set to 0");
            robottelemetry.update();
            // Stop all motion after Path is completed;
            motorLeftFront.setPower(0);
            motorRightFront.setPower(0);
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
        motorRightFront.setPower(rightPower);

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
        motorRightFront.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void stopRobot() {
        motorRightFront.setPower(0);
        motorLeftFront.setPower(0);
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
}
