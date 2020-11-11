package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import java.util.ArrayList;

public class AutoShootInfo {
    public ArrayList<Double> distances;
    public ArrayList<Double> rampAngles;
    public AutoShootInfo(){
        this.distances = new ArrayList<Double>();
        this.rampAngles = new ArrayList<Double>();
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

        rampAngles.add(0.63);
        rampAngles.add(0.59);
        rampAngles.add(0.59);
        rampAngles.add(0.58);
        rampAngles.add(0.59);
        rampAngles.add(0.59);
        rampAngles.add(0.58);
        rampAngles.add(0.58);
        rampAngles.add(0.55);
        rampAngles.add(0.52);
        rampAngles.add(0.48);
        rampAngles.add(0.45);
        rampAngles.add(0.42);
        rampAngles.add(0.39);
        rampAngles.add(0.35);
        rampAngles.add(0.32);

        for(int i = 0; i < distances.size(); i++){
            distances.set(i, distances.get(i) + 13.543);
        }
    }
}
