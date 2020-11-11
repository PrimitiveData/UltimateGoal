package org.firstinspires.ftc.teamcode.testclasses.WithHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Ramsete.PathEngine;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;

@TeleOp(name = "testRamsete", group = "TeleOp")
public class testRamsete extends LinearOpMode {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap, telemetry);
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        PathEngine ramesetePath = new PathEngine(40,5,"//sdcard//FIRST//Points.txt",hardware, this);
        ramesetePath.init();
        waitForStart();
        hardware.updatePID = true;
        hardwareThreadInterface.start();
        ramesetePath.run(hardware.time,20,0.7,false);
    }
}
