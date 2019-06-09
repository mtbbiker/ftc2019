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
    static final double     DRIVE_GEAR_REDUCTION    = 40 ;     // This is < 1.0 if geared UP 40:1 reduce to 160 rpm
    static final double     WHEEL_DIAMETER_MM   = 100 ;     // For figuring circumference
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);

    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.65;
    static final double     STRAFE_SPEED            = 0.5;

    private Telemetry robottelemetry;

    private BNO055IMU imu;

    private Orientation angles;
    private Acceleration gravity;

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
