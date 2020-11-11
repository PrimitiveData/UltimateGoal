package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="FFTest", group="TeleOp")
public class FFTest extends LinearOpMode {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap,telemetry);
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        double prevTime = time.milliseconds();
        double prevPosition = hardware.hub1Motors[0].motor.getCurrentPosition();
        hardware.hub1Motors[0].readRequested = true;
        hardware.sixWheelDrive.left.setState(0,0);
        hardware.sixWheelDrive.left.setState(0,0);
        double startTime = time.milliseconds();
        double ticker = 0;

        double requestedV = 0;
        FileWriter writer;
        try {
             writer = new FileWriter("//sdcard//FIRST//feedforwarddata.txt");
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        while(!isStopRequested()){
            hardware.loop();
            double currentTime = time.milliseconds();
            double velo = (hardware.localY)/((currentTime-prevTime)/1000)* Hardware.circumfrence/ Hardware.ticks_per_rotation;

            prevTime = currentTime;
            if(true){
                if(currentTime-startTime < 1000) {
                    RobotLog.dd("QFFTESTDEBUG",currentTime - startTime+"");
                    hardware.sixWheelDrive.left.setState((currentTime - startTime) / 1000 * 55,55);
                    hardware.sixWheelDrive.right.setState((currentTime - startTime) / 1000 * 55,55);
                    requestedV = (currentTime - startTime) / 1000 * 55;
                }
                else if(currentTime - startTime < 1500){
                    hardware.sixWheelDrive.left.setState(55,0);
                    hardware.sixWheelDrive.right.setState(55,0);
                    requestedV = 55;
                }
                else{

                    requestedV = 55+((currentTime - startTime) / 1000-1.5) * -55;
                    hardware.sixWheelDrive.left.setState(requestedV,-55);
                    hardware.sixWheelDrive.right.setState(requestedV,-55);
                }
            }
            double leftPower = hardware.sixWheelDrive.left.updateCurrentStateAndGetOutput(velo);
            hardware.sixWheelDrive.LB.motor.setPower(leftPower);
            hardware.sixWheelDrive.LF.motor.setPower(leftPower);
            double rightPower = hardware.sixWheelDrive.right.updateCurrentStateAndGetOutput(velo);
            hardware.sixWheelDrive.RB.motor.setPower(rightPower);
            hardware.sixWheelDrive.RF.motor.setPower(rightPower);
            RobotLog.dd("EFFTEST","Time: " + (currentTime-startTime)/1000+", RequestedV: " + requestedV + ", Velo: " + velo + ", outputPower: "+leftPower + " and " + rightPower + ", targetVelo: " + hardware.sixWheelDrive.right.desiredState + ", kV: " + hardware.sixWheelDrive.left.kV);
            try {
                writer.write("Time: " + (currentTime-startTime)/1000+", RequestedV: " + requestedV + ", Velo: " + velo + ", outputPower: "+leftPower + " and " + rightPower + ", targetVelo: " + hardware.sixWheelDrive.right.desiredState + ", kV: " + hardware.sixWheelDrive.left.kV + "\n");
            } catch (IOException e) {
                e.printStackTrace();
            }
            ticker++;
        }
        try {
            writer.close();
            hardware.sixWheelDrive.right.writer.close();
            hardware.sixWheelDrive.left.writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
