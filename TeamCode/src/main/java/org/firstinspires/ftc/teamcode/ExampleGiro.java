package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

//Objective is to Train Students how Autonomous Features work
//How to use the Giro to drive in a Straight line
@Autonomous(name = "ExampleGiro", group = "AutoOpTraining")
//@Disabled
public class ExampleGiro extends LinearOpMode {

    AutoOpRobot robot = new AutoOpRobot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData(">", "ExampleGiro: Press Play to start tracking");
        telemetry.update();

        waitForStart();
        robot.setRobottelemetry(telemetry);
        //Initiate the Gyro
        robot.start();

        robot.callibrateGyro();
        robot.resetGyro();

        // Loop until we're asked to stop
        while (opModeIsActive())  {

            //Assume the Sample heading was detected as 0.0 (Drive straight forward)
            //robot.imuTurn(1,17,3);
        }



    }
    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    void sendTelemetry()
    {
        telemetry.addData("Status", robot.getImu().getSystemStatus().toString());
        telemetry.addData("Calib", robot.getImu().getCalibrationStatus().toString());
        telemetry.addData("Heading", formatAngle(robot.getAngles().angleUnit, robot.getAngles().firstAngle));
        telemetry.addData("Roll", formatAngle(robot.getAngles().angleUnit, robot.getAngles().secondAngle));
        telemetry.addData("Pitch", formatAngle(robot.getAngles().angleUnit, robot.getAngles().thirdAngle));

        telemetry.addData("Grav", robot.getGravity().toString());
        telemetry.update();
    }


}
