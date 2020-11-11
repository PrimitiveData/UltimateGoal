package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

public abstract class AutoMethods extends LinearOpMode {
    public void turnTo(double targetAngleRadians, double duration, Hardware hardware){
        TurretPID headingPID = new TurretPID(1.2,6,0.12,Math.toRadians(20), hardware.time);
        headingPID.setState(Math.toRadians(targetAngleRadians));
        double startTime = hardware.time.milliseconds();
        while(!isStopRequested()&&hardware.time.milliseconds()-startTime<duration){
            double output = headingPID.updateCurrentStateAndGetOutput(hardware.angle);
            hardware.sixWheelDrive.turn(output);
            hardware.loop();
        }
        hardware.sixWheelDrive.turn(0);
    }
    public void goStraight(double power, int duration, Hardware hardware){
        hardware.sixWheelDrive.LF.setPower(power);
        hardware.sixWheelDrive.LB.setPower(power);
        hardware.sixWheelDrive.RF.setPower(power);
        hardware.sixWheelDrive.RB.setPower(power);
        sleep(duration);
        hardware.sixWheelDrive.LF.setPower(0);
        hardware.sixWheelDrive.LB.setPower(0);
        hardware.sixWheelDrive.RF.setPower(0);
        hardware.sixWheelDrive.RB.setPower(0);
    }
}
