package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;


import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.ContRotServo;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PID.PIDwithBasePower;
import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Turret {
    public static double ticks_per_radian=6258.22701028;
    Hardware hardware;
    ContRotServo[] turretServos;
    RegServo magRotationServo;
    public double startTurretPosition;
    public PIDwithBasePower turretPID;
    public Motor encoder;
    public double CENTER_TO_TURRET_INCHES;
    public boolean updatePID;
    public double maxCounterClockwise=180;
    public double maxClockwise=180;
    public double turretAngleOffsetAdjustmentConstant=0;
    AutoShootInfo info;
    public Turret(ContRotServo[] turretServos, RegServo magRotationServo, Motor encoder, Hardware hardware){
        this.magRotationServo = magRotationServo;
        this.turretServos = turretServos;
        this.hardware = hardware;
        this.encoder = encoder;
        encoder.readRequested = true;
        startTurretPosition = localTurretAngleRadians();
        //turretPID = new TurretPID(1,1,1,Math.toRadians(20),hardware.time);
        turretPID = new PIDwithBasePower(1.4,4.15,0.45,0.085,Math.toRadians(0.5), Math.toRadians(20), hardware.time);
        updatePID = false;
        info = new AutoShootInfo();
    }
    //gets the turret offset for shooting
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
        return turretAngleOffset+turretAngleOffsetAdjustmentConstant;
    }
    //sets the  angle of our turret to the global angle specified in the parameters
    public void setTurretAngle(double globalTurretAngle){//global turret angle is the angle with respect to the field, local is the angle with respect to the robot
        double desiredLocalTurretAngle = MathFunctions.keepAngleWithin180Degrees(globalTurretAngle - hardware.angle);
        if(desiredLocalTurretAngle > 100){
            desiredLocalTurretAngle = 100;
        }
        Hardware.telemetry.addData("desiredLocalTurretAngle", Math.toDegrees(desiredLocalTurretAngle));
        turretPID.setState(desiredLocalTurretAngle);
        double magAngle = (desiredLocalTurretAngle / Math.PI / 2) + 1/2;
        setMagAngle(magAngle);
    }
    //updates the turret's PID
    public void updateTurretPID(){
        double output = turretPID.updateCurrentStateAndGetOutput(localTurretAngleRadians());
        setAllTurretServoPowers(output);
    }
    //gets the position of the turret on the field
    public double[] getTurretPosition(){
        return MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),CENTER_TO_TURRET_INCHES,hardware.angle);
    }
    //sets the power of all the turret servos
    public void setAllTurretServoPowers(double power){
        for(ContRotServo crservo: turretServos){
            crservo.setPower(power);
        }
    }
    //points the robot directly towards the high goal
    public void pointTowardsHighGoal(){
        double[] currentPoint = getTurretPosition();
        double angleToPointTo = Math.atan2((currentPoint[1]- FieldConstants.highGoalPosition[1]),(currentPoint[0]-FieldConstants.highGoalPosition[0]));
        setTurretAngle(angleToPointTo);
    }
    //gets the local position of the turret in radians
    public double localTurretAngleRadians(){
        return -encoder.getCurrentPosition()/ticks_per_radian;
    }
    public void setMagAngle(double position) {
        magRotationServo.setPosition(position);
    }
}
