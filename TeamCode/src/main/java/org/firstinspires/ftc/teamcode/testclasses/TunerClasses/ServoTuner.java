package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="servotuner", group="TeleOp")
public class ServoTuner extends LinearOpMode {
    final double TICKS_PER_SEC = 0.2;
    public void runOpMode(){
        ElapsedTime time = new ElapsedTime();
        Servo tuned = hardwareMap.get(Servo.class,"tuned");
        waitForStart();

        double position = 0.5;
        double prevTime = time.milliseconds();
        while(!isStopRequested()) {
            double currentTime = time.milliseconds();
            double deltaTime = (currentTime-prevTime)/1000;
            prevTime = currentTime;
            position -= gamepad1.left_stick_y * TICKS_PER_SEC*deltaTime;
            if(position>1){
                position=1;
            }
            else if(position < 0){
                position=0;
            }
            telemetry.addData("pos: ",position);
            telemetry.update();
            tuned.setPosition(position);
        }

    }
}
