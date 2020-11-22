package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;


import java.util.ArrayList;

public class AutoShootInfo {
    public ArrayList<Double> distances;
    public ArrayList<Double> rampAngles;
    public ArrayList<Double> turretAngleOffsets;
    public ArrayList<Double> shooterSpeeds;
    public AutoShootInfo(){
        this.distances = new ArrayList<Double>();
        this.rampAngles = new ArrayList<Double>();
        this.turretAngleOffsets = new ArrayList<Double>();
        this.shooterSpeeds = new ArrayList<Double>();
        distances.add(52.0);
        distances.add(56.0);
        distances.add(60.0);
        distances.add(64.0);
        distances.add(68.0);
        distances.add(72.0);
        distances.add(76.0);
        distances.add(80.0);
        distances.add(84.0);
        distances.add(88.0);
        distances.add(92.0);
        distances.add(96.0);
        distances.add(100.0);
        distances.add(104.0);
        distances.add(108.0);
        distances.add(112.0);
        distances.add(116.0);
        distances.add(120.0);
        distances.add(124.0);

        rampAngles.add(0.71);
        rampAngles.add(0.67);
        rampAngles.add(0.63);
        rampAngles.add(0.6);
        rampAngles.add(0.57);
        rampAngles.add(0.54);
        rampAngles.add(0.5);
        rampAngles.add(0.46);
        rampAngles.add(0.42);
        rampAngles.add(0.39);
        rampAngles.add(0.36);
        rampAngles.add(0.32);
        rampAngles.add(0.27);
        rampAngles.add(0.22);
        rampAngles.add(0.22);
        rampAngles.add(0.19);
        rampAngles.add(0.19);
        rampAngles.add(0.19);
        rampAngles.add(0.19);

        turretAngleOffsets.add(-8.8);
        turretAngleOffsets.add(-8.6);
        turretAngleOffsets.add(-7.8);
        turretAngleOffsets.add(-7.6);
        turretAngleOffsets.add(-7.4);
        turretAngleOffsets.add(-7.2);
        turretAngleOffsets.add(-7.1);
        turretAngleOffsets.add(-6.6);
        turretAngleOffsets.add(-6.1);
        turretAngleOffsets.add(-5.7);
        turretAngleOffsets.add(-5.6);
        turretAngleOffsets.add(-5.4);
        turretAngleOffsets.add(-5.3);
        turretAngleOffsets.add(-5.2);
        turretAngleOffsets.add(-5.0);
        turretAngleOffsets.add(-4.8);
        turretAngleOffsets.add(-4.7);
        turretAngleOffsets.add(-5.6);
        turretAngleOffsets.add(-5.4);

        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1600.0);
        shooterSpeeds.add(-1450.0);
        shooterSpeeds.add(-1450.0);
        shooterSpeeds.add(-1450.0);
        shooterSpeeds.add(-1450.0);
        shooterSpeeds.add(-1450.0);


        for(int i = 0; i < distances.size(); i++){
            distances.set(i, distances.get(i) + 13.543);
        }

        for(int i = 0; i < turretAngleOffsets.size(); i++){
            turretAngleOffsets.set(i, Math.toRadians(turretAngleOffsets.get(i) - 2));

        }
    }
}
