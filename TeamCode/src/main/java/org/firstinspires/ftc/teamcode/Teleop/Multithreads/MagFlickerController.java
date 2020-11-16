package org.firstinspires.ftc.teamcode.Teleop.Multithreads;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Teleop.UltimateGoalTeleop;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;

import kotlin.jvm.internal.Intrinsics;

public class MagFlickerController extends Thread{
    public Hardware hardware;
    UltimateGoalTeleop parentOP;
    boolean shootRingRequested;
    boolean firstButtonPress = false;
    int numButtonPresses;
    String TAG = "MagFlickerController";
    public MagFlickerController(Hardware hardware, UltimateGoalTeleop parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
        shootRingRequested = false;
        firstButtonPress = true;
        numButtonPresses = 0;
    }
    public void sleeep(double milliseconds){
        double startTime = hardware.time.milliseconds();
        while(hardware.time.milliseconds() < startTime + milliseconds && !parentOP.teleopStopped){
            try{
                Thread.sleep(10);
            }catch(InterruptedException e){

            }
        }
    }
    public void run(){
        while(!parentOP.teleopStopped){
            if(shootRingRequested){
                numButtonPresses++;
                RobotLog.dd(TAG,numButtonPresses+" press checkpoint 1: " + hardware.time.milliseconds());
                hardware.mag.updateStateAndSetPosition();
                RobotLog.dd(TAG,numButtonPresses+" press checkpoint 2: " + hardware.mag.currentState.toString());
                if(firstButtonPress){
                    hardware.mag.currentState = Mag.State.TOP;
                    firstButtonPress = false;
                }
                RobotLog.dd(TAG,numButtonPresses+" press checkpoint 3: " + hardware.mag.currentState.toString());
                if(hardware.mag.currentState == Mag.State.COLLECT){
                    hardware.mag.setRingPusherResting();
                    RobotLog.dd(TAG,numButtonPresses+" press checkpoint 4: " + hardware.mag.currentState.toString());
                }
                else{
                    sleeep(500);
                    hardware.shooter.shooterVeloPID.speedyRecoveryOn = false;
                    hardware.mag.pushInRings();
                    RobotLog.dd(TAG,numButtonPresses+" press checkpoint 5 time: " + hardware.time.milliseconds());
                    sleeep(500);
                    hardware.shooter.shooterVeloPID.speedyRecoveryOn = true;
                    hardware.mag.setRingPusherResting();
                    RobotLog.dd(TAG,numButtonPresses+" press checkpoint 6 time: " + hardware.time.milliseconds());
                    sleeep(250);
                    if(hardware.mag.currentState == Mag.State.BOTTOM){
                        hardware.mag.updateStateAndSetPosition();
                    }
                }
                shootRingRequested = false;
            }
        }
    }
    public void updateMagStateAndSetPosition(){
        shootRingRequested = true;
    }
}
