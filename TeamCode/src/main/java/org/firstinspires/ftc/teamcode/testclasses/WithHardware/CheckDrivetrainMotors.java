package org.firstinspires.ftc.teamcode.testclasses.WithHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name = "checkDTmotors",group="TeleOp")
public class CheckDrivetrainMotors extends LinearOpMode {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap,telemetry);
        waitForStart();
        while(!isStopRequested()) {
            hardware.sixWheelDrive.RB.setPower(1);
            hardware.loop();
        }
    }
}
