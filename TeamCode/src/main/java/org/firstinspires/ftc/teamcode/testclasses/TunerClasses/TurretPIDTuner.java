package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.PID.PIDwithBasePower;

@Autonomous(name = "TurretPIDTuner", group="Autonomous")
public class TurretPIDTuner extends LinearOpMode {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap, telemetry);
        waitForStart();
        hardware.sendT265OdoData = false;
        PIDwithBasePower turretPID = new PIDwithBasePower(0.75/Math.toRadians(40),0.3,0.25,0.25,Math.toRadians(3),Math.toRadians(20), hardware.time);
        turretPID.setState(Math.toRadians(-90));
        while(!isStopRequested()) {
            telemetry.addData("heading: ", Math.toDegrees(hardware.turret.localTurretAngleRadians()));
            double output = turretPID.updateCurrentStateAndGetOutput(hardware.turret.localTurretAngleRadians());
            telemetry.addData("output: ",output);
            telemetry.addData("currentIntegral: ",turretPID.integral);
            telemetry.update();
            hardware.turret.setAllTurretServoPowers(output);
            hardware.loop();
        }
    }
}
