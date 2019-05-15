package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//Simple Mode to Detach from Lander to determine position for other AutoModes
@Autonomous(name = "AODetachTestLanding", group = "AutoOp")
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
        this.dropAndPark(-50,-20);
    }


}
