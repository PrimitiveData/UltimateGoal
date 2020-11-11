package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name="angularvelocontrollertest", group="TeleOp")
public class AngularVeloControllerTest extends LinearOpMode {
    Hardware hardware;
    String TAG = "feedfowardtuner";
    public void runOpMode(){
        hardware = new Hardware(hardwareMap,telemetry);
        waitForStart();
        hardware.updatePID = true;
        double prevAngle = 0;
        ElapsedTime time = new ElapsedTime();
        double prevTime = time.milliseconds();
        int ticker = 0;
        double addedAngularVelos = 0;
        while(!isStopRequested()){
            ticker++;
            hardware.sixWheelDrive.setMotion(10,0,40/(Hardware.trackWidth/2),0);
            telemetry.addData("requested angular velo (rad/s): ", 40/(Hardware.trackWidth/2));
            hardware.loop();
            double angle = hardware.angle;
            double currentTime = time.milliseconds();
            double anglularVelo = MathFunctions.keepAngleWithin180Degrees(angle-prevAngle)/((currentTime - prevTime)/1000);
            addedAngularVelos+=anglularVelo;
            telemetry.addLine("avg angular Velo: "+(addedAngularVelos/ticker));
            telemetry.addLine("angularVelo: "+(anglularVelo));
            telemetry.update();
            prevTime = currentTime;
            prevAngle = angle;
            sleep(50);
        }
    }
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
