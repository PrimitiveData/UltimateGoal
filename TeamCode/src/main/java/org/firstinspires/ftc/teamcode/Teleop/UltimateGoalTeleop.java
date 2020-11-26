package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.Teleop.Multithreads.MagFlickerController;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.WobblerArm;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;
import org.firstinspires.ftc.teamcode.hardware.PID.VelocityPID;
import org.firstinspires.ftc.teamcode.hardware.PID.VelocityPIDDrivetrain;

import java.io.IOException;

@TeleOp(name = "UltimateGoalTeleop",group="TeleOp")
public class UltimateGoalTeleop extends OpMode {
    double angleWhenIntakeIsOn = 0; // degrees
    Hardware hardware;
    boolean slowMode = false;
    boolean slowModeToggledPrevLoop = false;
    //toggles
    boolean manuelTurretControl = true;
    boolean manuelTurretControlToggledPrevLoop = false;
    boolean magUpdateStateAndSetPositionPrevLoop = false;
    boolean manuelRampControl = true;
    boolean manuelRampControlTogglePrevLoop = false;
    boolean shooterOn = false;
    boolean shooterOnTogglePrevLoop = false;
    boolean grip = true;
    boolean gripOnToggledPrevLoop = false;
    boolean gripperResting = true;
    boolean armStateToggledPrevLoop = false;
    public double shooterVelo;
    public boolean teleopStopped = false;

    boolean intakeOn = false;
    boolean intakeOnToggledPrevLoop = false;

    boolean dPadUpToggledPrevLoop = false;
    boolean dPadRightToggledPrevLoop = false;
    boolean dPadLeftToggledPrevLoop = false;
    boolean dPadDownToggledPrevLoop = false;
    MagFlickerController magFlickerController;
    boolean firstLoop;
    double startAngle;
    public boolean currentlyIncrementingMagDuringShooting;
    public void init(){
        /*if (T265.slamra == null) {
            T265.slamra = new T265Camera(new Transform2d(),T265.ODOMETRY_COVARIANCE, hardwareMap.appContext);
        }*/
        startAngle = Hardware.angleClassVariable;
        telemetry.addData("startAngle",startAngle);
        hardware = new Hardware(hardwareMap,telemetry);
        hardware.xPosTicks = Hardware.xPosTicksClassVariable;
        hardware.yPosTicks = Hardware.yPosTicksClassVariable;
        hardware.angle = startAngle;
        hardware.canglePrev = startAngle;
        slowMode = false;
        shooterVelo = -1600;
        magFlickerController = new MagFlickerController(hardware,this);
        hardware.mag.setRingPusherResting();
        hardware.wobbler.goToClawRestingPos();
        hardware.wobbler.goToArmRestingPos();
        firstLoop = true;
        currentlyIncrementingMagDuringShooting = false;
    }
    public double logistic(double input, double constantB, double constantC){
        return constantB*(1/(1+Math.pow(Math.E,-constantC*(input-0.6)))) - constantB/2+0.5532;
    }
    public void start(){
        //T265.slamra.start();
        magFlickerController.start();
    }
    public void loop(){
        double leftPower;
        double rightPower;
        if(gamepad1.left_trigger > 0) {
            if(!slowModeToggledPrevLoop) {
                slowMode = !slowMode;
            }
            slowModeToggledPrevLoop = true;
        }
        else{
            if(slowModeToggledPrevLoop){
                slowModeToggledPrevLoop = false;
            }
        }
        if(!slowMode) {
            double leftAbsValue = Math.abs(gamepad1.left_stick_y);
            double rightAbsValue = Math.abs(gamepad1.right_stick_y);
            leftPower = logistic(leftAbsValue, 1, 7.2) * -gamepad1.left_stick_y / leftAbsValue;
            rightPower = logistic(rightAbsValue, 1, 7.2) * -gamepad1.right_stick_y / rightAbsValue;
        }
        else{
            leftPower = -gamepad1.left_stick_y*0.5;
            rightPower = -gamepad1.right_stick_y*0.5;
        }
        if(gamepad1.b){
            leftPower=0.3;
            rightPower=-0.3;
        }
        hardware.sixWheelDrive.LF.setPower(leftPower);
        hardware.sixWheelDrive.LB.setPower(leftPower);
        hardware.sixWheelDrive.RF.setPower(rightPower);
        hardware.sixWheelDrive.RB.setPower(rightPower);
        hardware.sendT265OdoData= false;
        hardware.loop();
        /*T265Camera.CameraUpdate up = T265.slamra.getLastReceivedCameraUpdate();
        double[] t265position = T265.getCameraPosition(up);
        if(up.confidence == null){
            telemetry.addLine("no confidence level yet");
        }
        if(up.confidence == T265Camera.PoseConfidence.Failed){
            telemetry.addLine("Pose Confidence Failed");
        }
        else if(up.confidence == T265Camera.PoseConfidence.Medium){
            telemetry.addLine("Pose Confidence Medium");
        }
        else if(up.confidence == T265Camera.PoseConfidence.High){
            telemetry.addLine("Pose Confidence High");
        }
        else{
            telemetry.addLine("Pose Confidence Low");
        }*/
        //intake dropper
        if(gamepad1.y){
            hardware.intake.dropIntake();
        }
        //manuel turret control toggle & turret control
        if(gamepad2.a) {
            if(!manuelTurretControlToggledPrevLoop) {
                manuelTurretControl = !manuelTurretControl;
            }
            manuelTurretControlToggledPrevLoop = true;
        }
        else{
            if(manuelTurretControlToggledPrevLoop){
                manuelTurretControlToggledPrevLoop = false;
            }
        }
        if(manuelTurretControl){
            hardware.turret.updatePID = false;
            telemetry.addLine("manuel turret control on rn");
            hardware.turret.setAllTurretServoPowers(gamepad2.left_stick_x);
        }
        else{
            hardware.turret.updatePID = true;
            hardware.turret.pointTowardsHighGoal();
        }
        telemetry.addData("turret Position",hardware.turret.encoder.getCurrentPosition());
        //intake control
        if(gamepad1.right_trigger>0) {
            if(!intakeOnToggledPrevLoop) {
                intakeOn = !intakeOn;
            }
            intakeOnToggledPrevLoop = true;
        }
        else{
            if(intakeOnToggledPrevLoop){
                intakeOnToggledPrevLoop = false;
            }
        }
        if(intakeOn) {
            if(gamepad1.a){
                hardware.intake.turnIntake(-1);
            }else {
                hardware.intake.turnIntake(1);
            }
        }
        else{
            hardware.intake.turnIntake(0);
        }
        //mag control
        if(gamepad1.right_bumper) {
            if(!magUpdateStateAndSetPositionPrevLoop) {
                magFlickerController.updateMagStateAndSetPositionAndShootAllRings();
            }
            magUpdateStateAndSetPositionPrevLoop = true;
        }
        else{
            if(magUpdateStateAndSetPositionPrevLoop){
                magUpdateStateAndSetPositionPrevLoop = false;
            }
        }
        //ramp manuel control and automatic control
        if(gamepad1.x) {
            if(!manuelRampControlTogglePrevLoop) {
                manuelRampControl = !manuelRampControl;
            }
            manuelRampControlTogglePrevLoop = true;
        }
        else{
            if(manuelRampControlTogglePrevLoop){
                manuelRampControlTogglePrevLoop = false;
            }
        }
        if(manuelRampControl){
            hardware.turret.updatePID = false;
            hardware.shooter.setRampPosition(hardware.shooter.rampPostion - gamepad2.right_stick_y*0.001);
        }
        else{
            double[] turretPosition = MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),-4.72974566929,hardware.angle);
            telemetry.addLine("Turret Position: "+turretPosition[0]+", "+turretPosition[1]);
            double distanceToGoal = Math.hypot(turretPosition[1]- FieldConstants.highGoalPosition[1],turretPosition[0] - FieldConstants.highGoalPosition[0]);
            double angleToGoal = Math.atan2(FieldConstants.highGoalPosition[1]-turretPosition[1], FieldConstants.highGoalPosition[0]-turretPosition[0]) + hardware.turret.getTurretOffset(distanceToGoal);
            telemetry.addData("angleToGoal",Math.toDegrees(angleToGoal));
            if(!currentlyIncrementingMagDuringShooting) {
                hardware.shooter.autoRampPositionForHighGoal(distanceToGoal);
            }
            hardware.turret.updatePID = true;
            hardware.turret.setTurretAngle(angleToGoal);
            shooterVelo = hardware.shooter.autoaimShooterSpeed(distanceToGoal);
        }
        //shooter
        if(gamepad1.left_bumper) {
            if(!shooterOnTogglePrevLoop) {
                shooterOn = !shooterOn;
                if(shooterOn){
                    hardware.shooter.firstUpdateShooterPIDFLoop = true;
                }
            }
            shooterOnTogglePrevLoop = true;
            hardware.shooter.shooterVeloPID.clearI();
        }
        else{
            if(shooterOnTogglePrevLoop){
                shooterOnTogglePrevLoop = false;
            }
        }
        if(shooterOn){
            hardware.shooter.updatePID = true;
            /*double voltage = VelocityPIDDrivetrain.getBatteryVoltage();
            double maxVolts = -10.5;
            hardware.shooter.shooterMotor2.setPower(maxVolts/voltage);
            hardware.shooter.shooterMotor1.setPower(maxVolts/voltage);*/
            hardware.shooter.shooterVeloPID.setState(shooterVelo);
        }
        else{
            hardware.shooter.updatePID = false;
            hardware.shooter.shooterMotor2.setPower(-0.5);
            hardware.shooter.shooterMotor1.setPower(-0.5);
        }
        //Flicker
        if(gamepad2.x){
            hardware.mag.pushInRings();
        }
        if(gamepad2.y){
            hardware.mag.setRingPusherResting();
        }
        //wobbler
        if(gamepad2.left_bumper) {
            if(!gripOnToggledPrevLoop) {
                grip = !grip;
                gripperResting = false;
            }
            gripOnToggledPrevLoop = true;
        }
        else{
            if(gripOnToggledPrevLoop){
                gripOnToggledPrevLoop = false;
            }
        }
        if(gripperResting){
            hardware.wobbler.goToClawRestingPos();
        }
        else if(grip){
            hardware.wobbler.gripWobble();
        }else{
            hardware.wobbler.goToClawRestingPos();
        }
        if(gamepad2.right_bumper) {
            if(!armStateToggledPrevLoop) {
                hardware.wobbler.toggleArmState();
            }
            armStateToggledPrevLoop = true;
        }
        else{
            if(armStateToggledPrevLoop){
                armStateToggledPrevLoop = false;
            }
        }
        if(gamepad2.right_trigger>0){
            hardware.wobbler.armState = WobblerArm.ArmState.START;
            hardware.wobbler.goToArmRestingPos();
        }
        //end powershot
        if(gamepad1.dpad_left){
            manuelRampControl = true;
            hardware.mag.feedTopRing();
            hardware.mag.currentState = Mag.State.TOP;
            hardware.turret.turretPID.setState(Math.toRadians(-178));
            sleeep(1000);
            shootPowershot(hardware);
            telemetry.addLine("1st powershot");
            telemetry.update();
            hardware.turret.turretPID.setState(Math.toRadians(-183.5));
            sleeep(350);
            shootPowershot(hardware);
            telemetry.addLine("2nd powershot");
            telemetry.update();
            hardware.turret.turretPID.setState(Math.toRadians(-191.5));
            sleeep(250);
            shootPowershot(hardware);
            telemetry.addLine("3rd powershot");
            telemetry.update();
        }

        if(gamepad1.dpad_up){
            shooterVelo -= 0.5;
        }else if(gamepad1.dpad_down){
            shooterVelo += 0.5;
        }
        telemetry.addData("shooter On",shooterOn);
        if(hardware.mag.currentState == Mag.State.BOTTOM){
            telemetry.addLine("Mag State: BOTTOM");
        }
        else if(hardware.mag.currentState == Mag.State.MID){
            telemetry.addLine("Mag State: MID");
        }
        else if(hardware.mag.currentState == Mag.State.TOP){
            telemetry.addLine("Mag State: TOP");
        }
        else{
            telemetry.addLine("Mag State: COLLECT");
        }
        telemetry.addData("Wobbler grip",grip);
        telemetry.addData("Flap position",hardware.shooter.rampPostion);
        telemetry.addLine("angle: "+hardware.angle + ", in degrees: "+Math.toDegrees(hardware.angle) + ", from odo: "+ Math.toDegrees(hardware.angleOdo));
        telemetry.addLine("angle 1: "+Math.toDegrees(hardware.angle1) + ", angle 2: "+Math.toDegrees(hardware.angle2));
        telemetry.addLine("XCenter: " + hardware.getXAbsoluteCenter()  + ", YCenter: "+hardware.getYAbsoluteCenter());
        telemetry.addLine("left position: " + -hardware.hub1Motors[0].getCurrentPosition() + ", right position: " + -hardware.hub1Motors[3].motor.getCurrentPosition() + ", lateral position: " + hardware.hub1Motors[1].getCurrentPosition());
        telemetry.addLine("shooter velo: "+shooterVelo);
        telemetry.addLine("turret Angle: "+Math.toDegrees(hardware.turret.localTurretAngleRadians())+", turret output power: "+gamepad2.left_stick_x);
        telemetry.addLine("loops/sec: " + (hardware.loops / ((hardware.time.milliseconds()-hardware.startTime)/1000)));
        telemetry.update();
        //resetting odo

        if(gamepad2.dpad_up) {
            if(!dPadUpToggledPrevLoop) {
                hardware.yPosTicks -= 0.5* Hardware.ticks_per_rotation/ Hardware.circumfrence;
            }
            dPadUpToggledPrevLoop = true;
        }
        else{
            if(dPadUpToggledPrevLoop){
                dPadUpToggledPrevLoop = false;
            }
        }
        if(gamepad2.dpad_down) {
            if(!dPadDownToggledPrevLoop) {
                hardware.yPosTicks += 0.5* Hardware.ticks_per_rotation/ Hardware.circumfrence;
            }
            dPadDownToggledPrevLoop = true;
        }
        else{
            if(dPadDownToggledPrevLoop){
                dPadDownToggledPrevLoop = false;
            }
        }
        if(gamepad2.dpad_left) {
            if(!dPadLeftToggledPrevLoop) {
                hardware.xPosTicks += 0.5* Hardware.ticks_per_rotation/ Hardware.circumfrence;
            }
            dPadLeftToggledPrevLoop = true;
        }
        else{
            if(dPadLeftToggledPrevLoop){
                dPadLeftToggledPrevLoop = false;
            }
        }
        if(gamepad2.dpad_right) {
            if(!dPadRightToggledPrevLoop) {
                hardware.xPosTicks -= 0.5* Hardware.ticks_per_rotation/ Hardware.circumfrence;
            }
            dPadRightToggledPrevLoop = true;
        }
        else{
            if(dPadRightToggledPrevLoop){
                dPadRightToggledPrevLoop = false;
            }
        }
        if(firstLoop){
            hardware.angle = startAngle;
            hardware.canglePrev = startAngle;
            firstLoop=false;
        }
    }
    public void stop(){
        //T265.slamra.stop();
        teleopStopped = true;
        try {
            magFlickerController.writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void shootPowershot(Hardware hardware) {
        hardware.mag.pushInRings();
        sleeep(175);
        hardware.mag.setRingPusherResting();
        sleeep(200);
        hardware.mag.updateStateAndSetPosition();
    }
    public void turnTo(double targetAngleRadians, double duration, Hardware hardware) {
        TurretPID headingPID = new TurretPID(1.2, 6, 0.12, Math.toRadians(20), hardware.time);
        headingPID.setState(Math.toRadians(targetAngleRadians));
        double startTime = hardware.time.milliseconds();
        while (!teleopStopped && hardware.time.milliseconds() - startTime < duration) {
            double output = headingPID.updateCurrentStateAndGetOutput(hardware.angle);
            hardware.sixWheelDrive.turn(output);
            hardware.loop();
        }
        hardware.sixWheelDrive.turn(0);
    }
    public void sleeep(double milliseconds){
        double startTime = hardware.time.milliseconds();
        while(hardware.time.milliseconds() < startTime + milliseconds && !teleopStopped){
            try{
                Thread.sleep(10);
            }catch(InterruptedException e){

            }
        }
    }
}
