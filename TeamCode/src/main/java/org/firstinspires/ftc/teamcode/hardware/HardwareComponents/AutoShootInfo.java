package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import android.os.storage.StorageManager;

import java.util.ArrayList;

public class AutoShootInfo {
    public ArrayList<Double> distances;
    public ArrayList<Double> rampAngles;
    public ArrayList<Double> turretAngleOffsets;
    public AutoShootInfo(){
        this.distances = new ArrayList<Double>();
        this.rampAngles = new ArrayList<Double>();
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

        rampAngles.add(0.78);
        rampAngles.add(0.75);
        rampAngles.add(0.7);
        rampAngles.add(0.64);
        rampAngles.add(0.62);
        rampAngles.add(0.6);
        rampAngles.add(0.57);
        rampAngles.add(0.55);
        rampAngles.add(0.5);
        rampAngles.add(0.45);
        rampAngles.add(0.43);
        rampAngles.add(0.38);
        rampAngles.add(0.37);
        rampAngles.add(0.3);
        rampAngles.add(0.26);
        rampAngles.add(0.27);
        rampAngles.add(0.23);
        rampAngles.add(0.23);
        rampAngles.add(0.24);

        turretAngleOffsets.add(-8.8);
        turretAngleOffsets.add(-8.0);
        turretAngleOffsets.add(-7.2);
        turretAngleOffsets.add(-6.5);
        turretAngleOffsets.add(-6.4);
        turretAngleOffsets.add(-6.3);
        turretAngleOffsets.add(-6.0);
        turretAngleOffsets.add(-5.9);
        turretAngleOffsets.add(-5.7);
        turretAngleOffsets.add(-5.5);
        turretAngleOffsets.add(-5.4);
        turretAngleOffsets.add(-4.8);
        turretAngleOffsets.add(-5.5);
        turretAngleOffsets.add(-5.1);
        turretAngleOffsets.add(-5.0);
        turretAngleOffsets.add(-5.1);
        turretAngleOffsets.add(-4.8);
        turretAngleOffsets.add(-4.6);
        turretAngleOffsets.add(-4.5);

        for(int i = 0; i < distances.size(); i++){
            distances.set(i, distances.get(i) + 13.543);
        }

        for(int i = 0; i < turretAngleOffsets.size(); i++){
            turretAngleOffsets.set(i, Math.toRadians(turretAngleOffsets.get(i)));
        }
    }
}
