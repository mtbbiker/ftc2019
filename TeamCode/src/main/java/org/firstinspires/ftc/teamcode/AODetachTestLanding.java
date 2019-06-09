package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//Simple Mode to Detach from Lander to determine position for other AutoModes
@Disabled
@Autonomous(name = "Exp_DetachTestLanding", group = "AutoOp")
public class AODetachTestLanding extends AOPathBase {

    //AutoOpRobot robot = new AutoOpRobot();
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        robot.setRobottelemetry(telemetry);
        robot.start();
        this.unhitchRobot();

        this.setupCollectorliftarm();
        //Test Path 1 (L,C,R)
        //this.dropAndParkCratorSide(-50,-50);
        //Test Path 2 (L,C,R)
        //this.dropAndParkDepotSide(-50,-50);
    }


}
