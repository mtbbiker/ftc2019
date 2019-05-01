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

//Objective is to Train Students how Autonomous Features work
//How to use the Giro to drive in a Straight line
@Autonomous(name = "ExampleGiro", group = "AutoOpTraining")
public class ExampleGiro extends LinearOpMode {

    AutoOpRobot robot = new AutoOpRobot();



    @Override
    public void runOpMode() throws InterruptedException {



        robot.init(hardwareMap);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        robot.setRobottelemetry(telemetry);
        //Initiate the Gyro
        robot.callibrateGyro();
        robot.resetGyro();
        robot.start();


        // Loop until we're asked to stop
        while (opModeIsActive())  {

            //Drive in a Straight Line using the Gyro
            robot.encoderDriveStraight(1,500,5);
        }

    }


}
