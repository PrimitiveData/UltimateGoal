package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class HardwareThreadInterface extends Thread {
    public Hardware hardware;
    LinearOpMode parentOP;
    public HardwareMecanum hardwareMecanum;
    public HardwareThreadInterface(Hardware hardware, LinearOpMode parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
    }
    public HardwareThreadInterface(HardwareMecanum hardware, LinearOpMode parentOP){
        this.hardwareMecanum = hardware;
        this.parentOP = parentOP;
    }
    public void run(){
        while(!parentOP.isStopRequested()){
            if(hardware != null){
                hardware.loop();
            }
            else{
                hardwareMecanum.loop();
            }
            if(isInterrupted()){
                return;
            }
        }
    }
}
