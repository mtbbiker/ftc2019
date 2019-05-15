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

    public void setupCollectorliftarm(){

        robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorLift.setDirection(DcMotorSimple.Direction.REVERSE);

        //Lift and extend
        telemetry.addData("Lift Current Position",  "Target :%7d", robot.motorLift.getCurrentPosition());
        //Negative is Lift -700 Extender +3700
        robot.encoderExtender(1700,1,3);
        robot.encoderMoveLift(-600, 1, 3);

        telemetry.update();
    }

    //Path when Robot is facing the Crator from Latch
    //Drive from a predefined position, then drop Beacon and Park at the Crator
    //This is the Crator opposite from where the Robot started
    public void dropAndParkCratorSide(int initialturn, int backupPathLength){
        //Reverse a bit
        robot.encoderDriveStraight(1, backupPathLength, 4);
        //Turn left towards Depot
        robot.rotate((90 - initialturn),1);
        //Drive First Path towards wall
        robot.encoderDriveStraight(1,300,4);
        //Turn to Depot
        robot.rotate(45,1);
        //Drive to Depot
        robot.encoderDriveStraight(1,300,4);
        //Turn to Drop
        robot.rotate(-90,1);
        //Drop beacon
        robot.dropBeacon(2);
        //Turn Right again
        robot.rotate(-90,1);
        //Drive to crator
        robot.encoderDriveStraight(1,1500,5);
        //We might need to extend and lift, but we will Test first
    }

    //Path when Robot is facing the Depot from Latch
    //Drive from a predefined position, then drop Beacon and Park at the Crator
    //Left of the Depot looking from Lander (Depot left, Crator right)
    public void dropAndParkDepotSide(int initialturn, int backupPathLength){
        //Reverse a bit
        robot.encoderDriveStraight(1, backupPathLength, 4);
        //Turn left towards Depot
        robot.rotate((90 - initialturn),1);
        //Drive First Path towards wall
        robot.encoderDriveStraight(1,300,4);
        //Turn to Depot
        robot.rotate(-135,1);
        //Drive towards Depot
        robot.encoderDriveStraight(1,300,4);
        //Drop beacon
        robot.dropBeacon(2);
        //Turn Right to 1
        robot.rotate(-90,1);
        //Turn another to drive to Neutral Crator
        robot.rotate(-90,1);
        //Drive to crator
        robot.encoderDriveStraight(1,1500,5);
        //We might need to extend and lift, but we will Test first
    }
}
