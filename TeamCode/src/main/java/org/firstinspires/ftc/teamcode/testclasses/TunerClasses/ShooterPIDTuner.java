package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.io.IOException;

@TeleOp(name="ShooterPIDTuner", group="TeleOp")
public class ShooterPIDTuner extends LinearOpMode {
    public void runOpMode(){
        final double TIMETOREACHMAX = 10000;//milliseconds
        double  MAX_TICKS_PER_SEC =1432;//max ticks-per-sec
        Hardware hardware = new Hardware(hardwareMap,telemetry);
        waitForStart();
        hardware.shooter.shooterMotor1.readRequested = true;
        hardware.shooter.shooterMotor2.readRequested = true;
        double startTime = hardware.time.milliseconds();
        while(!isStopRequested()){
            double currentTime = hardware.time.milliseconds();
            hardware.shooter.updatePID = true;
            if(currentTime - startTime < TIMETOREACHMAX){
                hardware.shooter.shooterVeloPID.setState(-1/TIMETOREACHMAX*(currentTime-startTime) * MAX_TICKS_PER_SEC);
            }
            else{
                MAX_TICKS_PER_SEC -= gamepad1.left_stick_y;
                telemetry.addData("speed (Ticks/sec)", MAX_TICKS_PER_SEC);
                telemetry.update();
                hardware.shooter.shooterVeloPID.setState(-MAX_TICKS_PER_SEC);
            }
            hardware.loop();
        }
        try {
            hardware.shooter.shooterVeloPID.writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
