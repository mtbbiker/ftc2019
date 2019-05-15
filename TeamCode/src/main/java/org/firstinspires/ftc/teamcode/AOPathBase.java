package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class AOPathBase extends LinearOpMode {

    AutoOpRobot robot = new AutoOpRobot();

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void unhitchRobot() {
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

    //Drive from a predefined position, then drop Beacon and Park at the Crator
    public void dropAndPark(int initialturn, int backupPathLength){
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
}
