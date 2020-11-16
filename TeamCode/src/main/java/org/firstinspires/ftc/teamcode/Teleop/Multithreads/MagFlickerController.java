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
    public MagFlickerController(Hardware hardware, UltimateGoalTeleop parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
        shootRingRequested = false;
        firstButtonPress = true;
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
                hardware.mag.updateStateAndSetPosition();
                if(firstButtonPress){
                    hardware.mag.currentState = Mag.State.TOP;
                    firstButtonPress = false;
                }
                if(hardware.mag.currentState == Mag.State.COLLECT){
                    hardware.mag.setRingPusherResting();
                }
                else{
                    sleeep(500);
                    hardware.shooter.shooterVeloPID.speedyRecoveryOn = false;
                    hardware.mag.pushInRings();
                    sleeep(500);
                    hardware.shooter.shooterVeloPID.speedyRecoveryOn = true;
                    hardware.mag.setRingPusherResting();
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
