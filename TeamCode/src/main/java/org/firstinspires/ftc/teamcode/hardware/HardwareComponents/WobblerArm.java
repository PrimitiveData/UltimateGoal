package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class WobblerArm {
    RegServo wobblerArm;
    //wobbler arm positions
    public double armGrabWobblePos = 0.215;
    public double armPushWobblePos = 0.050116;
    public double armRaiseWobble = 0.55;
    public double armStartingPos = 0.74;
    public double armRestingPos = 0.87;
    RegServo wobblerClaw;
    public double clawReleasePos = 0.05;
    public double clawGrip = 0.35;
    public double clawRestingPos = 0.275;
    ArmState armState = ArmState.START;
    public WobblerArm(RegServo wobblerArm, RegServo wobblerClaw){
        this.wobblerArm = wobblerArm;
        this.wobblerClaw = wobblerClaw;
    }
    public void moveArmToGrabPos(){
        wobblerArm.setPosition(armGrabWobblePos);
    }
    public void gripWobble(){
        wobblerClaw.setPosition(clawGrip);
    }
    public void raiseWobble(){
        wobblerArm.setPosition(armRaiseWobble);
    }
    public void releaseWobble(){
        wobblerClaw.setPosition(clawReleasePos);
    }
    public void goToWobbleStartingPos(){
        wobblerArm.setPosition(armStartingPos);
    }
    public void goToAutoWobblerDropPosition(){wobblerArm.setPosition((armRaiseWobble+armGrabWobblePos)/2);}
    public enum ArmState{
        START,
        GRIP,
        LIFT;
    }
    public void toggleArmState(){
        if(armState == ArmState.START){
            armState = ArmState.GRIP;
        }
        else if(armState == ArmState.GRIP){
            armState = ArmState.LIFT;
        }
        else{
            armState = ArmState.GRIP;
        }
        if(armState == ArmState.GRIP){
            moveArmToGrabPos();
        }
        else if(armState == ArmState.LIFT){
            raiseWobble();
        }else{
            goToWobbleStartingPos();
        }
    }
}
