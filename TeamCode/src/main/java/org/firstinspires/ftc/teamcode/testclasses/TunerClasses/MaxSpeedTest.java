package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name="maxspeedTuner", group="TeleOp")
public class MaxSpeedTest extends LinearOpMode {
    Hardware hardware;
    String TAG = "feedfowardtuner";
    public void runOpMode(){
        hardware = new Hardware(hardwareMap,telemetry);
        waitForStart();
        hardware.sixWheelDrive.goStraight(1);
        double prevPos = hardware.hub1Motors[0].motor.getCurrentPosition() * Hardware.circumfrence / Hardware.ticks_per_rotation;
        ElapsedTime time = new ElapsedTime();
        double prevTime = time.milliseconds();
        while(!isStopRequested()){
            hardware.allHubs.get(0).clearBulkCache();
            sleep(10);
            double pos = hardware.hub1Motors[0].motor.getCurrentPosition() * Hardware.circumfrence / Hardware.ticks_per_rotation;
            RobotLog.dd("MaxSpeedDebug","Pos: "+pos + ", PrevPos: "+prevPos);
            RobotLog.dd("MaxSpeedDebug","position: " + hardware.hub1Motors[0].motor.getCurrentPosition());
            double currentTime = time.milliseconds();
            double velo = (pos-prevPos)/((currentTime-prevTime)/1000);
            prevTime = currentTime;
            RobotLog.dd("MaxSpeedTest","Velo: "+velo);
            prevPos = pos;
        }
    }
}
