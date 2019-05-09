package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;

//https://ftcforum.usfirst.org/forum/ftc-technology/53812-mounting-the-revhub-vertically

@TeleOp(name = "IMUtest", group = "Sensor")
//@Disabled
public class IMUtest extends LinearOpMode
{
    private BNO055IMU imu;

    private Orientation angles;
    private Acceleration gravity;

  //  Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    Orientation             lastAngles = new Orientation();
    //First is the matter of the axes reference:
    // INTRINSIC: is the coordinate system in which the referred-to rotational axes reside in a coordinate system that moves with
    // (and so remains fixed relative to) the object being rotated,
    // EXTRINSIC The axes remain fixed relative to the world around the object and are unaffected by the object's rotational motion?
    // Both points of view are equally valid methodologies, but one or the other may be more understandable or useful in a given application situation.
    // https://en.wikipedia.org/wiki/Euler_angles

    @Override
    public void runOpMode()
    {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //Default
        // {Z Y X} -> Z (10), Y (01), X (00) [0010 0100]bin => 0x024

        // Y and Z swapped
        // remap order {Z Y X} -> Y (01), Z (10), X (00) [0001 1000]bin => 0x018

//        // All swapped
//        //remap order {Z Y X} -> Y (01), X (00), Z (10) [0001 0010]bin => 0x012
//        byte AXIS_MAP_CONFIG_BYTE = 0x18; //0x12 // 06 This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
//        //Bit { X sign, Y sign, Z sign }
//        //Example all positive [000]bin => 0x00
//        //Change sign Z [001]bin
//        byte AXIS_MAP_SIGN_BYTE = 0x01; // 01 This is what to write to the AXIS_MAP_SIGN register to negate the z axis
//
//        //Need to be in CONFIG mode to write to registers
//        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
//
//        sleep(100); //Changing modes requires a delay before doing anything else
//
//        //Write to the AXIS_MAP_CONFIG register
//        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
//
//        //Write to the AXIS_MAP_SIGN register
//        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
//
//        //Need to change back into the IMU mode to use the gyro
//        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
//
//        sleep(100); //Changing modes again requires a delay
//
//        telemetry.setMsTransmissionInterval(100);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        //Reset Angles
//        resetAngle();

        // wait for start button.
        waitForStart();

        resetAngle();
        while (opModeIsActive())
        {
            //angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();

            //resetAngle();
            sendTelemetry();
            telemetry.addData("Rotate now","15 degrees left");
            sleep(1000);

            getAngle();
            sendTelemetry();
        }
    }

    void sendTelemetry()
    {
        telemetry.addData("Status", imu.getSystemStatus().toString());
        telemetry.addData("Calib", imu.getCalibrationStatus().toString());
        telemetry.addData("Heading R-", formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));
        telemetry.addData("Roll L+", formatAngle(lastAngles.angleUnit, lastAngles.secondAngle));
        telemetry.addData("Pitch U-", formatAngle(lastAngles.angleUnit, lastAngles.thirdAngle));

        telemetry.addData("Global Heading", formatAngle(lastAngles.angleUnit, globalAngle));
        telemetry.addData("Grav", gravity.toString());
        telemetry.update();
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
        //AxesReference indicates whether we have INTRINSIC rotations, where the axes move with the object that is rotating,
        // or EXTRINSIC rotations, where they remain fixed in the world around the object.
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation _angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = _angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = _angles;

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

}