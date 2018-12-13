package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.FtcUtils;
import org.firstinspires.ftc.teamcode.misc.RobotConstants;

/**
 * Created by Marco on 4/13/18.
 */
public class Robot {
    HardwareMap hwMap;
    ElapsedTime clock = new ElapsedTime();
    public IMU imu;
    public Sampler sampler;
    private Rev2mDistanceSensor frontDistanceSensor;
    private Rev2mDistanceSensor backDistanceSensor;
    private int encoderPos = 0;
    private int hangEncoderPos = 0;
    private int extendEncoderPos = 0;
    public boolean canSample = false;
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor nom = null;
    private DcMotor hang = null;
    private DcMotor extend = null;
    public double samplerTurnDegrees = 0;
    private DcMotor BL = null;
    private Servo nomServo1 = null;
    private Servo markerServo = null;
    private LinearOpMode context;
    private Servo nomServo2 = null;
    private DcMotor catapult = null;
    public void init(HardwareMap ahwMap, LinearOpMode context, boolean initSensors, boolean initVision) {
        this.context = context;
        hwMap = ahwMap;
        FR = hwMap.get(DcMotor.class, "FR");
        imu = new IMU();
        sampler = new Sampler();
        FL = hwMap.get(DcMotor.class, "FL");
        nom = hwMap.get(DcMotor.class, "nom");
        extend = hwMap.get(DcMotor.class, "extend");
        hang = hwMap.get(DcMotor.class, "hang");
        BR = hwMap.get(DcMotor.class, "BR");
        BL = hwMap.get(DcMotor.class, "BL");
        frontDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "front");
        backDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "back");
        markerServo = hwMap.get(Servo.class, "markerServo");
        catapult = hwMap.get(DcMotor.class, "catapult");
        catapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nom.setDirection(DcMotorSimple.Direction.FORWARD);
        hang.setDirection(DcMotorSimple.Direction.REVERSE);
        extend.setDirection(DcMotorSimple.Direction.FORWARD);
        catapult.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        resetTicks();
        if (initSensors) {
            imu.init(hwMap, "imu");
            imu.resetAngle();
        }
        if (initVision) {
            canSample = sampler.init(hwMap, this.context);
        }
    }
    public void init(HardwareMap ahwMap, LinearOpMode context, boolean initSensors) {
        init(ahwMap, context, initSensors, false);
    }
    public void init(HardwareMap ahwMap, LinearOpMode context) {
        init(ahwMap, context, false, false);
    }
    public void moveTicks(int ticks, double pow, int timeout) {
        resetTicks();
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        double newPow = FtcUtils.map(FtcUtils.abs(ticks) - FtcUtils.abs(getTicks()), 0, FtcUtils.abs(ticks), RobotConstants.LOWEST_MOTOR_POWER, pow);
        while (FtcUtils.abs(getTicks()) < FtcUtils.abs(ticks) && currentTime - startTime < timeout && context.opModeIsActive()) {
            drive(FtcUtils.sign(ticks) * FtcUtils.abs(newPow), FtcUtils.sign(ticks) * FtcUtils.abs(newPow), FtcUtils.sign(ticks) * FtcUtils.abs(newPow), FtcUtils.sign(ticks) * FtcUtils.abs(newPow));
            currentTime = System.currentTimeMillis();
            context.telemetry.addData("Target", ticks);
            context.telemetry.addData("Current", getTicks());
            context.telemetry.addData("newPow", newPow);
            context.telemetry.update();
            newPow = FtcUtils.map(FtcUtils.abs(ticks) - FtcUtils.abs(getTicks()), 0, FtcUtils.abs(ticks), RobotConstants.LOWEST_MOTOR_POWER, pow);
        }
        stop();
        context.telemetry.addData("status", "done");
        context.telemetry.update();
    }
    public void strafeTicks(int ticks, double pow, int timeout) {
        resetTicks();
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        double newPow = FtcUtils.map(FtcUtils.abs(ticks) - FtcUtils.abs(getTicks()), 0, FtcUtils.abs(ticks), RobotConstants.LOWEST_STRAFE_POWER, pow);
         while (FtcUtils.abs(getTicks()) < FtcUtils.abs(ticks) && currentTime - startTime < timeout && context.opModeIsActive()) {
            drive(FtcUtils.sign(ticks) * FtcUtils.abs(newPow), -FtcUtils.sign(ticks) * FtcUtils.abs(newPow), FtcUtils.sign(ticks) * FtcUtils.abs(newPow), -FtcUtils.sign(ticks) * FtcUtils.abs(newPow));
            currentTime = System.currentTimeMillis();
            context.telemetry.addData("Target", ticks);
            context.telemetry.addData("Current", getTicks());
            context.telemetry.addData("newPow", newPow);
            context.telemetry.update();
            newPow = FtcUtils.map(FtcUtils.abs(ticks) - FtcUtils.abs(getTicks()), 0, FtcUtils.abs(ticks), RobotConstants.LOWEST_STRAFE_POWER, pow);
        }
        stop();
        context.telemetry.addData("status", "done");
        context.telemetry.update();
    }
    private void strafeTicksNoStopping(int ticks, double pow, int timeout) {
        resetTicks();
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        while (FtcUtils.abs(getTicks()) < FtcUtils.abs(ticks) && currentTime - startTime < timeout && context.opModeIsActive()) {
            drive(FtcUtils.sign(ticks) * FtcUtils.abs(pow), -FtcUtils.sign(ticks) * FtcUtils.abs(pow), FtcUtils.sign(ticks) * FtcUtils.abs(pow), -FtcUtils.sign(ticks) * FtcUtils.abs(pow));
            currentTime = System.currentTimeMillis();
            context.telemetry.addData("Target", ticks);
            context.telemetry.addData("Current", getTicks());
            context.telemetry.addData("pow", pow);
            context.telemetry.update();
        }
        context.telemetry.addData("status", "done");
        context.telemetry.update();
    }
    public void drive(double fl, double bl, double fr, double br) {
        FL.setPower(fl);
        BL.setPower(bl);
        FR.setPower(fr);
        BR.setPower(br);
    }
    public void drive(double fl, double bl, double fr, double br, int time) {
        FL.setPower(fl);
        BL.setPower(bl);
        FR.setPower(fr);
        BR.setPower(br);
        context.sleep(time);
        stop();
    }
    public void rotate(double degs, double pow, int timeout) {
        imu.resetAngle();
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        double currentAngle = imu.getAngle();
        double newPow = pow;
        context.telemetry.addData("status", "waiting for start");
        context.telemetry.addData("newPow", newPow);
        context.telemetry.addData("globalAngle", currentAngle);
        context.telemetry.addData("global less than degs", FtcUtils.abs(currentAngle) < FtcUtils.abs(degs));
        context.telemetry.update();
        while (FtcUtils.abs(currentAngle) < FtcUtils.abs(degs) && currentTime - startTime < timeout && context.opModeIsActive()) {
            currentAngle = imu.getAngle();
            drive(-FtcUtils.sign(degs) * newPow, -FtcUtils.sign(degs) * newPow, FtcUtils.sign(degs) * newPow, FtcUtils.sign(degs) * newPow);
            newPow = FtcUtils.map(FtcUtils.abs(degs) - FtcUtils.abs(currentAngle), 0, FtcUtils.abs(degs), RobotConstants.LOWEST_TURN_POWER, pow);
            context.telemetry.addData("cur pow", newPow);
            context.telemetry.addData("cur angle", currentAngle);
            context.telemetry.addData("angle diff", FtcUtils.abs(degs) - FtcUtils.abs(currentAngle));
            context.telemetry.update();
            currentTime = System.currentTimeMillis();
            imu.updateAngle();
        }
        stop();
        context.telemetry.addData("status", "done");
        context.telemetry.update();
    }
    public void rotatePID(double degs, double pow, int timeout) {
        imu.resetAngle();
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        double currentAngle = imu.getAngle();
        double newPow = pow;
        context.telemetry.addData("status", "waiting for start");
        context.telemetry.addData("newPow", newPow);
        context.telemetry.addData("globalAngle", currentAngle);
        context.telemetry.addData("global less than degs", FtcUtils.abs(currentAngle) < FtcUtils.abs(degs));
        context.telemetry.update();
        while (FtcUtils.abs(degs) - FtcUtils.abs(currentAngle) > RobotConstants.TURN_TOLERANCE && currentTime - startTime < timeout && context.opModeIsActive()) {
            currentAngle = imu.getAngle();
            drive(-newPow, -newPow, newPow, newPow);
            context.telemetry.addData("cur pow", newPow);
            context.telemetry.addData("cur angle", currentAngle);
            context.telemetry.addData("angle diff", FtcUtils.abs(degs) - FtcUtils.abs(currentAngle));
            newPow = FtcUtils.sign(degs) * newPow - getKp() * (degs - currentAngle);
            context.telemetry.update();
            currentTime = System.currentTimeMillis();
            imu.updateAngle();
        }
        stop();
        context.telemetry.addData("status", "done");
        context.telemetry.update();
    }
    public void deploy() {
        hangTicks(RobotConstants.MAX_HANG_TICKS, 1, 10000);
        if (canSample) samplerTurnDegrees = getSamplerTurnDegrees(2500);
        drive(.5, -.5, .5, -.5, 300);
        context.sleep(100);
        moveTicks(-100, .4, 2000);
        context.sleep(100);
        strafeTicks(400, .6, 2000);
        context.sleep(100);
        moveTicks(100, .4, 2000);
        context.sleep(100);
    }
    public void alignWithWall(double pow) {
        double frontDist = frontDist();
        double backDist = backDist();
        double initialDist = Math.abs(frontDist - backDist);
        double newPow = pow;
        while (Math.abs(frontDist - backDist) > 1.0 && context.opModeIsActive()) {
            drive(-FtcUtils.sign(frontDist - backDist) * newPow, -FtcUtils.sign(frontDist - backDist) * newPow, FtcUtils.sign(frontDist - backDist) * newPow, FtcUtils.sign(frontDist - backDist) * newPow);
            context.telemetry.addData("front", frontDist);
            context.telemetry.addData("back", backDist);
            context.telemetry.addData("initial diff", initialDist);
            context.telemetry.addData("curr diff", frontDist - backDist);
            context.telemetry.update();
            newPow = FtcUtils.map(Math.abs(frontDist - backDist), 0, initialDist, RobotConstants.LOWEST_TURN_POWER, pow);
            frontDist = frontDist();
            backDist = backDist();
        }
        stop();
    }

    public void moveToCrater() {
        moveTicks(-300, .8, 5000);
        strafeTicks(1200, .9, 3000);
        strafeTicks(-300, .7, 1500);
        moveTicks(-1500, .8, 5000);
        strafeTicks(1200, .7, 3000);
        strafeTicks(-130, .7, 3000);
        moveTicks(-1100, .8, 1500);
    }
    public void dropTeamMarker() {
        markerServo(RobotConstants.MARKERSERVO_DROP);
        context.sleep(600);
        markerServo(RobotConstants.MARKERSERVO_HOLD);
    }
    public void hangTicks(int ticks, double pow, int timeout) {
        runEncoderMotor(hang, ticks, pow, timeout);
    }
    public void extendTicks(int ticks, double pow, int timeout) {
        runEncoderMotor(extend, ticks, pow, timeout);
    }
    public boolean canExtendUp() {
        return getExtendTicks() <= RobotConstants.MAX_EXTEND_TICKS && context.gamepad2.left_stick_y < 0;
    }
    public boolean canExtendDown() {
        return getExtendTicks() >= RobotConstants.MIN_EXTEND_TICKS && context.gamepad2.left_stick_y > 0;
    }
    public boolean canExtend() {
        return canExtendUp() || canExtendDown();
    }
    public boolean canHangDown() {
        return getHangTicks() > -RobotConstants.MAX_HANG_TICKS;
    }
    public boolean canHangUp() {
        return getHangTicks() < 0;
    }
    private void runEncoderMotor(DcMotor motor, int ticks, double pow, int timeout) {
        int encoderPos = motor.getCurrentPosition();
        int currentPos = motor.getCurrentPosition() - encoderPos;
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        while (FtcUtils.abs(currentPos) < FtcUtils.abs(ticks) && currentTime - startTime < timeout && context.opModeIsActive()) {
            motor.setPower(pow);
            currentTime = System.currentTimeMillis();
            context.telemetry.addData("Target", ticks);
            context.telemetry.addData("Current", currentPos);
            context.telemetry.update();
            currentPos = motor.getCurrentPosition() - encoderPos;
        }
        motor.setPower(0);
        context.telemetry.addData("status", "done");
        context.telemetry.update();
    }
    public double getSamplerTurnDegrees(int timeout) {
        switch (sampler.getPosition(timeout)) {
            case LEFT:
                return 35.0;
            case RIGHT:
                return -35.0;
            case CENTER:
                return 0.0;
            default:
                return 0.0;
        }
    }
    public void resetTicks() {
        encoderPos = BL.getCurrentPosition();
        hangEncoderPos = hang.getCurrentPosition();
        extendEncoderPos = extend.getCurrentPosition();
    }
    public int getTicks() {
        return BL.getCurrentPosition() - encoderPos;
    }
    public int getHangTicks() {
        return hang.getCurrentPosition() - hangEncoderPos;
    }
    public int getExtendTicks() {
        return -(extend.getCurrentPosition() - extendEncoderPos);
    }
    public void nom(double power) {
        nom.setPower(power);
    }
    public void catapult(double power) {
        catapult.setPower(power);
    }
    public void hang(double power) {
        hang.setPower(power);
    }
    public void extend(double power) {
        extend.setPower(power);
    }
    public void nomServo(double pos) {
        if (nomServo2 == null) {
            nomServo2 = hwMap.get(Servo.class, "nomServo2");
            nomServo2.setDirection(Servo.Direction.REVERSE);
        }
        nomServo2.setPosition(pos);
    }
    public void markerServo(double pos) {
        markerServo.setPosition(pos);
    }
    public double nomServoPos() {
        if (nomServo2 == null ) {
            nomServo2 = hwMap.get(Servo.class, "nomServo2");
            nomServo2.setDirection(Servo.Direction.REVERSE);
        }
        return nomServo2.getPosition();
    }
    public double frontDist() {
        return frontDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public double backDist() {
        return backDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public void stop() {
        drive(0, 0, 0, 0);
    }
    double getKp() {
        return 0.02;
    }
}