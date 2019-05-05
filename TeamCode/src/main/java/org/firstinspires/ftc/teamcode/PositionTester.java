package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@Autonomous(name = "EncoderTester", group = "Tester")
public class PositionTester extends LinearOpMode {
    public DcMotor aMotor;

    //HardwareMap masterConfig = null;

    @Override
    public void runOpMode() throws InterruptedException {

        boolean isStarted =false;

        hardwareMap.get(DcMotor.class, "motorExtend");

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(!isStarted){
            telemetry.addData("Test Counts for debugging of Motor Encoder",  "Running at %7d", aMotor.getCurrentPosition());
            telemetry.update();
        }

    }

//    public void init(HardwareMap amasterConfig) {
//masterConfig = amasterConfig;
//        masterConfig
//        amasterConfig.get(DcMotor.class, "aMotor");
//    }

}
