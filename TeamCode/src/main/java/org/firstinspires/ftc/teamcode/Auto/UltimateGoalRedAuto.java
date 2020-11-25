package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.AutoAim;
import org.firstinspires.ftc.teamcode.Auto.Multithreads.MoveArmDownAfterDropping1stWobbler;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.Ramsete.PathEngine;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;
import org.firstinspires.ftc.teamcode.vision.UltimateGoalReturnPositionPipeline;

@Autonomous(name = "RedAuto", group = "Autonomous")
public class UltimateGoalRedAuto extends AutoMethods {
    int stack = 2;
    public void runOpMode(){
        OpenCvCamera webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        waitForStart();
        UltimateGoalReturnPositionPipeline pipeline = new UltimateGoalReturnPositionPipeline();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        webcam.resumeViewport();

        Hardware hardware = new Hardware(hardwareMap, telemetry);
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        PathEngine goToShootPos = new PathEngine(40,5,"//sdcard//FIRST//UGauto//goToShootPos.txt",hardware,this);
        PathEngine dropWobbler1;
        if(stack==0) {
            dropWobbler1=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler1Stack0.txt", hardware, this);
        }
        else if(stack == 1){
            dropWobbler1=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler1Stack1.txt", hardware, this);
        }
        else{
            dropWobbler1=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler1Stack2.txt", hardware, this);
        }
        PathEngine collect2ndWobbler;
        if(stack==0) {
            collect2ndWobbler=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//collect2ndWobblerStack0.txt", hardware, this);
        }
        else if(stack == 1){
            collect2ndWobbler=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//collect2ndWobblerStack1.txt", hardware, this);
        }else{
            collect2ndWobbler=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//collect2ndWobblerStack2.txt", hardware, this);
        }

        PathEngine dropWobbler2;
        if(stack==0) {
            dropWobbler2 = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler2Stack0.txt", hardware, this);
        }else if(stack==1){
            dropWobbler2 = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler2Stack1.txt", hardware, this);
        }else{
            dropWobbler2 = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler2Stack2.txt", hardware, this);
        }

        PathEngine park = null;
        if(stack==0) {
            park = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//parkStack0.txt", hardware, this);
        }
        else if(stack == 2){
            park = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//parkStack2.txt", hardware, this);
        }
        PathEngine intakeStack = null;
        if(stack==2){
            intakeStack = new PathEngine(40,5,"//sdcard//FIRST//UGauto//intakeStackStack2.txt",hardware,this);
        }
        /*
        PathEngine park;
        if(stack == 0){
            park = new PathEngine(40,5,"//sdcard//FIRST//UGauto//parkStack0.txt",hardware,this);
        }else if(stack == 1){
            park = new PathEngine(40,5,"//sdcard//FIRST//UGauto//parkStack1.txt",hardware,this);
        }else{
            park = new PathEngine(40,5,"//sdcard//FIRST//UGauto//parkStack2.txt",hardware,this);
        }*/
        hardware.wobbler.goToWobbleStartingPos();
        hardware.wobbler.gripWobble();
        hardware.loop();
        goToShootPos.init();
        collect2ndWobbler.init();
        dropWobbler1.init();
        dropWobbler2.init();
        if(stack == 0||stack==2) {
            park.init();
        }
        if(stack == 2){
            intakeStack.init();
        }
        hardware.mag.setRingPusherResting();
        hardware.mag.currentState = Mag.State.TOP;
        hardware.mag.magServo.servo.setPosition(0.32);
        while(!isStarted()){
            telemetry.addLine("Stack: "+pipeline.stack);
            telemetry.update();
            stack = pipeline.stack;
        }
        webcam.closeCameraDevice();
        //first powershot
        hardware.turret.turretPID.leewayDistance = Math.toRadians(1);
        hardwareThreadInterface.start();
        hardware.shooter.setRampPosition(0);
        hardware.shooter.shooterVeloPID.setState(-1600);
        hardware.shooter.updatePID = true;
        hardware.turret.turretPID.setState(Math.toRadians(-178));
        hardware.turret.updatePID = true;
        goToShootPos.run(hardware.time,20,0.7,false);
        hardware.mag.feedTopRing();
        sleep(150);
        shootPowershot(hardware);
        telemetry.addLine("1st powershot");
        telemetry.update();
        hardware.turret.turretPID.setState(Math.toRadians(-183));
        sleep(350);
        shootPowershot(hardware);
        telemetry.addLine("2nd powershot");
        telemetry.update();
        hardware.turret.turretPID.setState(Math.toRadians(-189));
        sleep(250);
        shootPowershot(hardware);
        telemetry.addLine("3rd powershot");
        telemetry.update();
        //2nd powershot
        /*
        hardware.mag.setRingPusherResting();
        hardware.mag.updateStateAndSetPosition();
        turnTo(Math.toRadians(-3),1500,hardware);
        hardware.mag.pushInRings();
        sleep(500);
        //3rd powershot
        hardware.mag.setRingPusherResting();
        hardware.mag.updateStateAndSetPosition();
        turnTo(Math.toRadians(-6),1500,hardware);
        hardware.mag.pushInRings();
        sleep(500);

        turnTo(-90,1500,hardware);


        double[] turretPosition = MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),-4.72974566929,hardware.angle);
        double distanceToGoal = Math.hypot(turretPosition[1]- FieldConstants.highGoalPosition[1],turretPosition[0] - FieldConstants.highGoalPosition[0]);
        double angleToGoal = Math.atan2(FieldConstants.highGoalPosition[1]-turretPosition[1], FieldConstants.highGoalPosition[0]-turretPosition[0]) - hardware.turret.getTurretOffset(distanceToGoal);
        hardware.shooter.autoRampPositionForHighGoal(distanceToGoal);
        hardware.turret.updatePID = true;
        hardware.turret.setTurretAngle(angleToGoal);
        for(int i = 0; i < 2; i++){
            hardware.mag.updateStateAndSetPosition();
            sleep(750);
            hardware.mag.pushInRings();
            sleep(500);
            if(i == 1){
                break;
            }
            hardware.mag.setRingPusherResting();
            sleep(250);
        }*/
        hardware.shooter.updatePID = false;
        hardware.turret.updatePID = false;
        hardware.turret.setAllTurretServoPowers(0);
        hardware.intake.turnIntake(0);

        if(stack==1) {
            turnTo(-14, 1000, hardware);
        }
        else if(stack == 2){
            //turnTo(-27,1000,hardware);
        }
        else{
            turnTo(-59.38139458892,1250,hardware);
        }
        hardware.updatePID = true;
        hardware.turret.updatePID = true;
        hardware.turret.turretPID.setState(0);
        dropWobbler1.run(hardware.time,20,0.7,false);
        hardware.wobbler.moveArmToGrabPos();
        sleep(500);
        hardware.wobbler.releaseWobble();
        sleep(500);
        hardware.wobbler.goToWobbleStartingPos();

        if(stack==1){
            goStraight(0.4,50,hardware);
            turnTo(-90,1000,hardware);
        }
        else if(stack == 2){
            goStraight(0.4,50,hardware);
            turnTo(-143.14,1000,hardware);
        }
        else{
            goStraight(0.4,50,hardware);
            turnTo(-162.054604099,1500,hardware);
        }
        if(stack==1) {
            MoveArmDownAfterDropping1stWobbler moveArmDownAfterDropping1stWobbler = new MoveArmDownAfterDropping1stWobbler(hardware, this);
            moveArmDownAfterDropping1stWobbler.start();
        }
        else{
            hardware.wobbler.moveArmToGrabPos();
        }
        if(stack == 0){
            collect2ndWobbler.run(hardware.time,40,0.7,false);
        }else if(stack==1){
            collect2ndWobbler.run(hardware.time, 20, 0.7, false);
        }else{
            collect2ndWobbler.run(hardware.time,40,0.7,false);
        }
        hardware.wobbler.gripWobble();
        sleep(250);
        hardware.wobbler.raiseWobble();
        sleep(250);
        if(stack == 0){
            turnTo(-360-16.260204696416,1250,hardware);
        }else if(stack == 1) {
            turnTo(-360, 1000, hardware);
        }else{
            turnTo(-108,1100,hardware);

            hardware.intake.turnIntake(1);
            hardware.turret.updatePID = true;
            hardware.shooter.updatePID = true;
            AutoAim autoAim = new AutoAim(hardware,telemetry,this);
            autoAim.run();
            goStraightEncoder(0.5,4.5,hardware);
            sleep(800);
            goStraightEncoder(0.5,2,hardware);
            sleep(800);
            goStraightEncoder(0.5,1.5,hardware);
            hardware.shooter.shooterVeloPID.setState(-1600);

            sleep(3000);
            hardware.mag.currentState = Mag.State.TOP;
            hardware.mag.feedTopRing();
            sleep(200);
            shootPowershot(hardware);
            sleep(250);
            shootPowershot(hardware);
            sleep(300);
            shootPowershot(hardware);
            autoAim.stopRequested = true;
            hardware.turret.updatePID=false;
            hardware.shooter.updatePID=false;
            while(!isStopRequested()) {
                telemetry.addData("X",hardware.getX());
                telemetry.addData("Y",hardware.getY());
                telemetry.update();
            }
            turnTo(-10.46,750,hardware);
        }

        if(stack==0){
            dropWobbler2.run(hardware.time,60,0.7,false);
        }
        else if(stack == 1) {
            dropWobbler2.run(hardware.time, 20, 0.7, false);
        }
        else{
            dropWobbler2.run(hardware.time,60,0.7,false);
        }
        hardware.wobbler.goToAutoWobblerDropPosition();
        turnTo(0,750,hardware);
        hardware.wobbler.releaseWobble();
        sleep(250);
        hardware.wobbler.goToWobbleStartingPos();
        if(stack == 0) {
            turnTo(-270, 1000, hardware);
            park.run(hardware.time,20,0.7,false);
        }
        else if(stack == 2){
            park.run(hardware.time,20,0.7,true);
        }

    }
}
