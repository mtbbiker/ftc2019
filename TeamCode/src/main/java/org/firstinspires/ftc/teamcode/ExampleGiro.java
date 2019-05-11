package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    //AutoOpRobot robot = new AutoOpRobot();
    CloneAutoOpRobot robot = new CloneAutoOpRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData(">", "ExampleGiro: Press Play to start tracking");
        telemetry.update();

        waitForStart();

        robot.setRobottelemetry(telemetry);
        //robot.start();

        robot.motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Initiate the Gyro
        //robot.callibrateGyro();
        //robot.resetGyro();
        robot.start(); //This actuall start the Opmode and set the "opModeIsActive" to TRUE

//        if(opModeIsActive()){
//            while (opModeIsActive()){
            //Assume the Sample heading was detected as 0.0 (Drive straight forward)
            telemetry.addData(">Active<", "Turning");

            //robot.imuTurn(0.5,17,3);
            telemetry.addData("Turning, ", "Direction 22 degrees" );
            telemetry.update();

            //robot.rotate(22,0.1);
            //telemetry.addData("Turned, ", "Heading" + robot.getAngle());

            telemetry.addData("Drive Straight, ", "600 mm" );
            telemetry.update();

            //robot.imuDriveStraight(0.75,600,3);
            //robot.encoderDriveStraight(0.5,300,3);
            robot.encoderDriveForwardorBackwards(0.5, 950, 7);
            telemetry.addData("Get Position, ", "Moved 300mm");
            telemetry.update();
//
//                sleep(2000);
//                break;
//
//            }
//        }
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
//        telemetry.addData("Status", robot.getImu().getSystemStatus().toString());
//        telemetry.addData("Calib", robot.getImu().getCalibrationStatus().toString());
//        telemetry.addData("Heading", formatAngle(robot.getAngles().angleUnit, robot.getAngles().firstAngle));
//        telemetry.addData("Roll", formatAngle(robot.getAngles().angleUnit, robot.getAngles().secondAngle));
//        telemetry.addData("Pitch", formatAngle(robot.getAngles().angleUnit, robot.getAngles().thirdAngle));
//
//        telemetry.addData("Grav", robot.getGravity().toString());
        telemetry.update();
    }


}
