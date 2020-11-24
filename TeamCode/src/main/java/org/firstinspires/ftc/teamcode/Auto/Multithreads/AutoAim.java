package org.firstinspires.ftc.teamcode.Auto.Multithreads;

import android.telecom.TelecomManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class AutoAim extends Thread {
    Hardware hardware;

    Telemetry telemetry;
    LinearOpMode parentOP;
    public boolean stopRequested;
    public AutoAim(Hardware hardware, Telemetry telemetry,LinearOpMode parentOP){
        this.telemetry = telemetry;
        this.hardware = hardware;
        this.parentOP = parentOP;
        stopRequested = false;
    }
    public void start(){
        while(!parentOP.isStopRequested()&&!stopRequested) {
            double[] turretPosition = MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(), hardware.getYAbsoluteCenter(), -4.72974566929, hardware.angle);
            telemetry.addLine("Turret Position: " + turretPosition[0] + ", " + turretPosition[1]);
            double distanceToGoal = Math.hypot(turretPosition[1] - FieldConstants.highGoalPosition[1], turretPosition[0] - FieldConstants.highGoalPosition[0]);
            double angleToGoal = Math.atan2(FieldConstants.highGoalPosition[1] - turretPosition[1], FieldConstants.highGoalPosition[0] - turretPosition[0]) + hardware.turret.getTurretOffset(distanceToGoal);
            telemetry.addData("angleToGoal", Math.toDegrees(angleToGoal));
            hardware.shooter.autoRampPositionForHighGoal(distanceToGoal);
            hardware.turret.setTurretAngle(angleToGoal);
        }
    }
}