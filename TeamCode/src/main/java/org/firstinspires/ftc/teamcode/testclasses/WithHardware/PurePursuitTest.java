package org.firstinspires.ftc.teamcode.testclasses.WithHardware;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.PurePursuit.PPrunner;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;
import org.firstinspires.ftc.teamcode.vision.T265;

@TeleOp(name = "pptest", group = "TeleOp")
public class PurePursuitTest extends LinearOpMode {
    public void runOpMode(){
        if (T265.slamra == null) {
            T265.slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(new HardwareMecanum(hardwareMap,telemetry), this);
        HardwareMecanum hardware = hardwareThreadInterface.hardwareMecanum;
        PPrunner spline = new PPrunner("//sdcard//FIRST//pointspp.txt",hardware,this);
        waitForStart();
        hardwareThreadInterface.start();
        spline.start(1,4,0,gamepad1);
        hardwareThreadInterface.interrupt();
        while(!isStopRequested()){
            hardware.mecanumDrive.setPowers(-gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x);
            hardware.loop();
            telemetry.addLine("left position: " + hardware.previousPortReading + ", right position: " + hardware.previousStarboardReading + ", lateral position: " + hardware.previousLateralReading);
            telemetry.addLine("angle: "+hardware.angle + ", in degrees: "+Math.toDegrees(hardware.angle) + ", from odo: "+ Math.toDegrees(hardware.angleOdo));
            telemetry.addLine("angle 1: "+Math.toDegrees(hardware.angle1) + ", angle 2: "+Math.toDegrees(hardware.angle2));
            telemetry.addLine("X: " + hardware.getX()+ ", Y: "+ hardware.getY() + ", angle: " + hardware.angle);
            telemetry.addLine("XAlt: " + hardware.xPosTicksAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAlt: "+hardware.yPosTicksAlt* Hardware.circumfrence/ Hardware.ticks_per_rotation);
            telemetry.addLine("XAlt2: " + hardware.xPosTicksAlt2 * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAlt2: "+hardware.yPosTicksAlt2* Hardware.circumfrence/ Hardware.ticks_per_rotation);
            telemetry.addLine("XAltAlt: "+ hardware.xPosTicksAltAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAltAlt: " + hardware.yPosTicksAltAlt * Hardware.circumfrence/ Hardware.ticks_per_rotation);
            telemetry.addLine("angularVeloTracker: "+hardware.integratedAngularVeloTracker);
            telemetry.addLine("loops/sec: " + (hardware.loops / ((hardware.time.milliseconds()-hardware.startTime)/1000)));
        }
        T265.slamra.stop();
    }
}
