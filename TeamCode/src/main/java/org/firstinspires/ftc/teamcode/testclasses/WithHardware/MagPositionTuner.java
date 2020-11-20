package org.firstinspires.ftc.teamcode.testclasses.WithHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
@TeleOp(name = "magPositionTuner",group="TeleOp")
public class MagPositionTuner extends LinearOpMode {
    final double TICKS_PER_SEC = 0.2;
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware(hardwareMap,telemetry);
        double currentMagPosition = 0.5;
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        double prevTime = time.milliseconds();
        while(!isStopRequested()) {
            double currentTime = time.milliseconds();
            double deltaTime = (currentTime-prevTime)/1000;
            prevTime = currentTime;
            currentMagPosition -= gamepad1.left_stick_y * TICKS_PER_SEC*deltaTime;
            if(currentMagPosition>1){
                currentMagPosition=1;
            }
            else if(currentMagPosition < 0){
                currentMagPosition=0;
            }
            telemetry.addData("pos: ",currentMagPosition);
            telemetry.update();
            hardware.mag.magServo.servo.setPosition(currentMagPosition);
        }
    }
}
