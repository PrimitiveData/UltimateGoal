package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testEncoder",group = "TeleOp")
public class testEncoder extends LinearOpMode {
    public void runOpMode(){
        DcMotor toTest = hardwareMap.get(DcMotor.class,"toTest");
        waitForStart();
        while(!isStopRequested()){
            telemetry.addData("toTestPosition",toTest.getCurrentPosition());
            telemetry.update();
        }
    }
}
