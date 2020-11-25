package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

public abstract class AutoMethods extends LinearOpMode {
    public void turnTo(double targetAngleRadians, double duration, Hardware hardware) {
        TurretPID headingPID = new TurretPID(1.2, 6, 0.12, Math.toRadians(20), hardware.time);
        headingPID.setState(Math.toRadians(targetAngleRadians));
        double startTime = hardware.time.milliseconds();
        while (!isStopRequested() && hardware.time.milliseconds() - startTime < duration) {
            double output = headingPID.updateCurrentStateAndGetOutput(hardware.angle);
            hardware.sixWheelDrive.turn(output);
            hardware.loop();
        }
        hardware.sixWheelDrive.turn(0);
    }

    public void goStraight(double power, int duration, Hardware hardware) {
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

    public void shootPowershot(Hardware hardware) {
        hardware.mag.pushInRings();
        sleep(175);
        hardware.mag.setRingPusherResting();
        sleep(200);
        hardware.mag.updateStateAndSetPosition();
    }
    public void goStraightEncoder(double power, double distance, Hardware hardware){
        double startStarboard = -hardware.hub1Motors[3].getCurrentPosition();
        double startPort = -hardware.hub1Motors[0].getCurrentPosition();
        while(((-hardware.hub1Motors[3].getCurrentPosition() - startStarboard) + (-hardware.hub1Motors[0].getCurrentPosition() - startPort))/2 < distance*Hardware.ticks_per_rotation/Hardware.circumfrence){
            telemetry.addLine("startStarboard: "+startStarboard);
            telemetry.addLine("startPort: "+startPort);
            telemetry.addLine("currentStarboard" + hardware.hub1Motors[3].getCurrentPosition());
            telemetry.addLine("currentPort" + hardware.hub1Motors[0].getCurrentPosition());
            telemetry.addLine("portDiff: "+(hardware.hub1Motors[0].getCurrentPosition()-startPort));
            telemetry.addLine("starboardDiff: "+(hardware.hub1Motors[3].getCurrentPosition()-startStarboard));
            telemetry.update();
            hardware.sixWheelDrive.LF.setPower(power);
            hardware.sixWheelDrive.LB.setPower(power);
            hardware.sixWheelDrive.RF.setPower(power);
            hardware.sixWheelDrive.RB.setPower(power);
        }

        hardware.sixWheelDrive.LF.setPower(0);
        hardware.sixWheelDrive.LB.setPower(0);
        hardware.sixWheelDrive.RF.setPower(0);
        hardware.sixWheelDrive.RB.setPower(0);
    }
}
