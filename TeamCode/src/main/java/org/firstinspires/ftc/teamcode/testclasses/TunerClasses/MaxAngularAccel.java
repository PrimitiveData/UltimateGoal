package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="maxangularaccel", group="TeleOp")
public class MaxAngularAccel extends LinearOpMode {
    Hardware hardware;
    String TAG = "feedfowardtuner";
    public double accelTime = 500;
    public void runOpMode(){
        hardware = new Hardware(hardwareMap,telemetry);
        waitForStart();
        double prevVelo = 0;
        double prevPos = Math.toRadians(hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        FileWriter writer;
        try {
            writer = new FileWriter("//sdcard//FIRST//feedforwarddata.txt");
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        ElapsedTime time = new ElapsedTime();
        double startTime = time.milliseconds();
        double currentTime = startTime;
        double prevTime = startTime;
        while(!isStopRequested() && currentTime < startTime + accelTime){
            hardware.allHubs.get(0).clearBulkCache();
            currentTime = time.milliseconds();
            double timeDiff = currentTime - startTime;
            double i = timeDiff/accelTime;
            hardware.sixWheelDrive.turn(i);
            telemetry.addData("power",i);
            double currentPos = Math.toRadians(hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            double velocity = MathFunctions.keepAngleWithin180Degrees(currentPos - prevPos)/((currentTime-prevTime)/1000);
            double accel = (velocity-prevVelo)/((currentTime-prevTime)/1000);
            prevTime = currentTime;
            prevVelo = velocity;
            prevPos = currentPos;
            RobotLog.dd(TAG, "Velo: " + velocity + ", Power: " + i + ", Accel: "+ accel);
            try {
                writer.write("Velo: " + velocity + ", Power: " + i + ", Accel: "+ accel + ", Voltage: " + i*getBatteryVoltage() +"\n");
            } catch (IOException e) {
                e.printStackTrace();
            }

        }
        try {
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
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
