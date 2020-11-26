package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.AutoMethods;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;

@Autonomous(name = "goStraightTest", group="auto")
public class goStraight extends AutoMethods {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap,telemetry);
        HardwareThreadInterface hardwareThreadInterface = new HardwareThreadInterface(hardware,this);
        waitForStart();
        hardware.updatePID=false;
        hardwareThreadInterface.start();
        goStraightEncoder(-0.5,-50,hardware);
    }
}
