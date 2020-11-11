package org.firstinspires.ftc.teamcode.Teleop.Multithreads;

import org.firstinspires.ftc.teamcode.Teleop.UltimateGoalTeleop;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;

public class MagFlickerController extends Thread{
    public Hardware hardware;
    UltimateGoalTeleop parentOP;
    boolean shootRingRequested;
    public MagFlickerController(Hardware hardware, UltimateGoalTeleop parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
    }
    public void run(){
        while(!parentOP.teleopStopped){
            if(shootRingRequested){
                hardware.mag.updateStateAndSetPosition();
                if(hardware.mag.currentState == Mag.State.COLLECT){
                    hardware.mag.setRingPusherResting();
                }
                else{
                    try{
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    hardware.shooter.shooterVeloPID.speedyRecoveryOn = false;
                    hardware.mag.pushInRings();
                    try{
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    hardware.shooter.shooterVeloPID.speedyRecoveryOn = true;
                    hardware.mag.setRingPusherResting();
                    try{
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
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
