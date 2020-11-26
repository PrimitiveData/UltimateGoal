package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;


import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.ContRotServo;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PID.PIDwithBasePower;

public class Turret {
    public static double ticks_per_radian=15833.3703586*34/45/2*188/180;
    Hardware hardware;
    ContRotServo[] turretServos;
    public double startTurretPosition;
    public PIDwithBasePower turretPID;
    public Motor encoder;
    public double CENTER_TO_TURRET_INCHES;
    public boolean updatePID;
    public double maxCounterClockwise=180;
    public double maxClockwise=180;
    AutoShootInfo info;
    public Turret(ContRotServo[] turretServos, Motor encoder, Hardware hardware){
        this.turretServos = turretServos;
        this.hardware = hardware;
        this.encoder = encoder;
        encoder.readRequested = true;
        startTurretPosition = localTurretAngleRadians();
        //turretPID = new TurretPID(1,1,1,Math.toRadians(20),hardware.time);
        turretPID = new PIDwithBasePower(0.97,1.2,0.31,0.09,Math.toRadians(0.75),Math.toRadians(20), hardware.time);
        updatePID = false;
        info = new AutoShootInfo();
    }
    public double getTurretOffset(double distanceToGoal){
        double turretAngleOffset = 0;
        for(int i = 0; i < info.distances.size()-1; i++){
            if(MathFunctions.isInBetween(info.distances.get(i), info.distances.get(i+1), distanceToGoal)){
                double slope = (info.turretAngleOffsets.get(i+1) - info.turretAngleOffsets.get(i))/(info.distances.get(i+1)-info.distances.get(i));
                turretAngleOffset = slope*(distanceToGoal - info.distances.get(i))+info.turretAngleOffsets.get(i);
            }
        }
        if(distanceToGoal > info.distances.get(info.distances.size()-1)){
            turretAngleOffset = info.turretAngleOffsets.get(info.turretAngleOffsets.size()-1);
        }
        if(distanceToGoal < info.distances.get(0)){
            turretAngleOffset = info.turretAngleOffsets.get(0);
        }
        return turretAngleOffset;
    }
    public void setTurretAngle(double globalTurretAngle){//global turret angle is the angle with respect to the field, local is the angle with respect to the robot
        double desiredLocalTurretAngle = MathFunctions.keepAngleWithin180Degrees(globalTurretAngle - hardware.angle);
        if(desiredLocalTurretAngle > 100){
            desiredLocalTurretAngle = 100;
        }
        Hardware.telemetry.addData("desiredLocalTurretAngle",desiredLocalTurretAngle);
        turretPID.setState(desiredLocalTurretAngle);
    }
    public void updateTurretPID(){
        double output = turretPID.updateCurrentStateAndGetOutput(localTurretAngleRadians());
        setAllTurretServoPowers(output);
    }
    public double[] getTurretPosition(){
        return MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),CENTER_TO_TURRET_INCHES,hardware.angle);
    }
    public void setAllTurretServoPowers(double power){
        for(ContRotServo crservo: turretServos){
            crservo.setPower(power);
        }
    }
    public void pointTowardsHighGoal(){
        double[] currentPoint = getTurretPosition();
        double angleToPointTo = Math.atan2((currentPoint[1]- FieldConstants.highGoalPosition[1]),(currentPoint[0]-FieldConstants.highGoalPosition[0]));
        setTurretAngle(angleToPointTo);
    }
    public double localTurretAngleRadians(){
        return -encoder.getCurrentPosition()/ticks_per_radian;
    }
}
