package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
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
    private ModernRoboticsI2cRangeSensor frontDistanceSensor;
    private ModernRoboticsI2cRangeSensor backDistanceSensor;
    private int encoderPos = 0;
    private int hangEncoderPos = 0;
    private int extendEncoderPos = 0;
    private int liftEncoderPos = 0;
    public boolean canSample = false;
    private RobotConstants.AutoType autoType;
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor nom = null;
    private DcMotor hang = null;
    private DcMotor extend = null;
    public double samplerTurnDegrees = 0;
    private DcMotor BL = null;
    PwmControl placementPWM;
    PwmControl nomPWM;
    private Servo nomRotator = null;
    private Servo placementRotator = null;
    private Servo markerServo = null;
    private LinearOpMode context;
    private DcMotor lift = null;
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
        //frontDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "front");
        //backDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "back");
        placementRotator = hwMap.get(Servo.class, "placementRotator");
        nomRotator = hwMap.get(Servo.class, "nomRotator");
        lift = hwMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nom.setDirection(DcMotorSimple.Direction.FORWARD);
        hang.setDirection(DcMotorSimple.Direction.FORWARD);
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
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
    public void init(HardwareMap ahwMap, LinearOpMode context, boolean initSensors, boolean initVision, RobotConstants.AutoType autoType) {
        this.autoType = autoType;
        init(ahwMap, context, initSensors, initVision);
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
            drive(FtcUtils.sign(ticks) * FtcUtils.abs(newPow), -FtcUtils.sign(ticks) * FtcUtils.abs(newPow), -FtcUtils.sign(ticks) * FtcUtils.abs(newPow), FtcUtils.sign(ticks) * FtcUtils.abs(newPow));
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
            newPow = FtcUtils.motorScale(FtcUtils.sign(degs) * RobotConstants.LOWEST_TURN_POWER - getKp() * (degs - currentAngle));
            context.telemetry.update();
            currentTime = System.currentTimeMillis();
            imu.updateAngle();
        }
        stop();
        context.telemetry.addData("status", "done");
        context.telemetry.update();
    }
    public void deploy() {
        placementRotator.setPosition(RobotConstants.PLACEMENTSERVO_RECEIVE);
        hangTicks(RobotConstants.MAX_HANG_TICKS, 1, 10000);
        if (canSample) samplerTurnDegrees = getSamplerTurnDegrees(2500);
        drive(.5, -.5, .5, -.5, 300);
        moveTicks(-100, .5, 800);
        strafeTicks(75, .7, 900);
        moveTicks(100, .5, 800);
        rotate(90, .6, 3500);
    }
    public void alignWithWall(double pow) {
        double newPow = pow;
        double initialDist = distDifference();
        double diff = distDifference();
        while (Math.abs(diff) > 1.5 && context.opModeIsActive()) {
            drive(-FtcUtils.sign(diff) * newPow, -FtcUtils.sign(diff) * newPow, FtcUtils.sign(diff) * newPow, FtcUtils.sign(diff) * newPow);
            context.telemetry.addData("initial diff", initialDist);
            context.telemetry.addData("curr diff", diff);
            context.telemetry.update();
            newPow = FtcUtils.map(Math.abs(diff), 0, initialDist, RobotConstants.LOWEST_TURN_POWER, pow);
            diff = distDifference();
        }
        stop();
    }

    public void moveToWall() {
        if (samplerTurnDegrees == 35.0) rotate(75 - samplerTurnDegrees + 15, .6, 2500);
        else if (samplerTurnDegrees == -27.0) {
            if (autoType == RobotConstants.AutoType.DEPOT) rotate(75 - samplerTurnDegrees - 14, .6, 2050);
            if (autoType == RobotConstants.AutoType.CRATER) rotate(75 - samplerTurnDegrees - 23, .6, 2050);
        }
        else if(samplerTurnDegrees == 0.0) {
            if (autoType == RobotConstants.AutoType.DEPOT) rotate(71, .65, 2500);
            else rotate(65, .65, 2500);
        }
        if (samplerTurnDegrees == 35.0) moveTicks(-1500, .6, 2500);
        else if (samplerTurnDegrees == -27.0) {
            if (autoType == RobotConstants.AutoType.DEPOT) moveTicks(-1240, .6, 2500);
            else moveTicks(-1700, .6, 2500);
        } else if (samplerTurnDegrees == 0.0) moveTicks(-1350, .6, 2500);
        if (samplerTurnDegrees == 35.0) rotate(70, .6, 2500);
        else if (samplerTurnDegrees == -27.0) {
            if (autoType == RobotConstants.AutoType.DEPOT) rotate(55, .6, 2500);
            if (autoType == RobotConstants.AutoType.CRATER) rotate(56, .6, 2500);
        } else if (samplerTurnDegrees == 0.0) {
            rotate(58, .6, 2500);
        }
        //strafeTicks(-100, .6, 500);
        if (autoType == RobotConstants.AutoType.DEPOT) {
            if (samplerTurnDegrees == -27.0) {
                strafeTicks(-1000, .7, 2000);
            } else {
                strafeTicks(-600, .7, 1000);
            }
            strafeTicks(150, .6, 500);
        } else {
            strafeTicks(-600, .7, 1000);
            strafeTicks(150, .6, 500);
        }
    }
    public void placeTeamMarker() {
        if (autoType == RobotConstants.AutoType.DEPOT) {
            moveTicks(-350, .5, 1500);
            extendTicks(1700, 1, 3000);
            nomRotator(RobotConstants.NOMSERVO_DOWN + .15);
            context.sleep(600);
            nom(-.8);
            context.sleep(600);
            nom(0);
            nomRotator(RobotConstants.NOMSERVO_UP);
            extendTicks(-1700, 1, 3000);
            moveTicks(350, .5, 1500);
        } else if (autoType == RobotConstants.AutoType.CRATER) {
            if (samplerTurnDegrees == 0) moveTicks(-1300, .5, 3000);
            if (samplerTurnDegrees == 35.0) moveTicks(-1850, .5, 3000);
            if (samplerTurnDegrees == -27.0) moveTicks(-1300, .5, 3000);
            nomRotator(RobotConstants.NOMSERVO_DOWN + .15);
            context.sleep(600);
            nom(-.4);
            context.sleep(600);
            nom(0);
            nomRotator(RobotConstants.NOMSERVO_UP);
        }
    }
    public double liftPower() {
        return lift.getPower();
    }
    public void moveToCrater() {
        if (autoType == RobotConstants.AutoType.DEPOT) {
            moveTicks(-900, .5, 2200);
            nomRotator(RobotConstants.NOMSERVO_NEUTRAL);
        }
        if (autoType == RobotConstants.AutoType.CRATER) {
            if (samplerTurnDegrees == -27.0) moveTicks(1300, .5, 2200);
            else if (samplerTurnDegrees == 0) moveTicks(1800, .5, 2200);
            else moveTicks(1700, .5, 2200);
            strafeTicks(-500, .7, 1600);
            strafeTicks(100, .6, 600);
            if (samplerTurnDegrees == -27.0) moveTicks(1200, .5, 2200);
            else moveTicks(500, .6, 2200);
        }
        while (context.opModeIsActive()) context.idle();
    }
    public void sample() {
        int sampleMoveTicks = 1000;
        if (canSample) {
            if (autoType == RobotConstants.AutoType.CRATER) sampleMoveTicks = 850;
            rotate(samplerTurnDegrees, .5, 2000);
            if (samplerTurnDegrees == 0) strafeTicks(70, .6, 500);
            // starting garbage thing
            /*nomRotator(RobotConstants.NOMSERVO_DOWN);
            nom(-1);
            context.sleep(600);
            extendTicks(1250, 1, 3000);
            context.sleep(600);
            nom(0);
            nomRotator(RobotConstants.NOMSERVO_NEUTRAL);
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            int extendPos2 = extend.getCurrentPosition();
            extendTicks(-10, .1, 3000);
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
            //ending garbage thing
            moveTicks(-sampleMoveTicks, .5, 3000);
            context.sleep(500);
            if (samplerTurnDegrees ==  -27.0) {
                moveTicks(sampleMoveTicks - 200, .5, 3000);
            } else {
                moveTicks(sampleMoveTicks, .5, 3000);
            }
            rotate(-samplerTurnDegrees, .5, 2000);
        }
    }
    public void hangTicks(int ticks, double pow, int timeout) {
        runEncoderMotor(hang, ticks, pow, timeout);
    }
    public void extendTicks(int ticks, double pow, int timeout) {
        runEncoderMotor(extend, ticks, -pow, timeout);
    }
    public void liftTicks(int ticks, double pow, int timeout) {
        runEncoderMotor(lift, ticks, -pow, timeout);
    }
    public boolean canExtendOut() {
        return getExtendTicks() <= RobotConstants.MAX_EXTEND_TICKS && context.gamepad2.left_stick_y < 0;
    }
    public boolean canExtendIn() {
        return getExtendTicks() >= RobotConstants.MIN_EXTEND_TICKS && context.gamepad2.left_stick_y > 0;
    }
    public boolean canLiftUp() {
        return getLiftTicks() <= RobotConstants.MAX_LIFT_TICKS && context.gamepad2.left_stick_y < 0;
    }
    public boolean canLiftDown() {
        return getLiftTicks() >= RobotConstants.MIN_LIFT_TICKS && context.gamepad2.left_stick_y  > 0;
    }
    public boolean canLift() {
        return canLiftUp() || canLiftDown();
    }
    public boolean canExtend() {
        return canExtendOut() || canExtendIn();
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
            motor.setPower(FtcUtils.sign(ticks) * pow);
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
                return -27.0;
            case CENTER:
                return 0.0;
            default:
                return 0.0;
        }
    }
    public double placementRotatorPos() {
        return placementRotator.getPosition();
    }
    public void moveAngle(int ticks, double pow, double angle, int timeout) {
        double m = 0;
        double vx = pow*Math.cos(angle*(Math.PI/180.0)+Math.PI/4);
        double vy = pow*Math.sin(angle*(Math.PI/180.0)+Math.PI/4);
        double newvx = vx;
        double newvy = vy;
        if (Math.abs(vx) > m)
            m = vx;
        if (Math.abs(vy) > m)
            m = vy;
        vx = Math.abs(pow) * vx / Math.abs(m);
        vy = Math.abs(pow) * vy / Math.abs(m);
        resetTicks();
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        while (FtcUtils.abs(getTicks()) < FtcUtils.abs(ticks) && currentTime - startTime < timeout && context.opModeIsActive()) {
            drive(newvx, newvy, newvy, newvx);
            drive(FtcUtils.sign(ticks) * FtcUtils.abs(newvx), FtcUtils.sign(ticks) * FtcUtils.abs(newvy), FtcUtils.sign(ticks) * FtcUtils.abs(newvy), FtcUtils.sign(ticks) * FtcUtils.abs(newvx));
            currentTime = System.currentTimeMillis();
            context.telemetry.addData("Target", ticks);
            context.telemetry.addData("Current", getTicks());
            context.telemetry.update();
            newvx = FtcUtils.map(FtcUtils.abs(ticks) - FtcUtils.abs(getTicks()), 0, FtcUtils.abs(ticks), RobotConstants.LOWEST_STRAFE_POWER, vx);
            newvy = FtcUtils.map(FtcUtils.abs(ticks) - FtcUtils.abs(getTicks()), 0, FtcUtils.abs(ticks), RobotConstants.LOWEST_STRAFE_POWER, vy);
        }
        stop();
        context.telemetry.addData("status", "done");
        context.telemetry.update();
    }
    public void moveAngle(double pow, double angle, int time) {
        double m = 0;
        double vx = pow*Math.cos(angle*(Math.PI/180.0)+Math.PI/4);
        double vy = pow*Math.sin(angle*(Math.PI/180.0)+Math.PI/4);
        if (Math.abs(vx) > m)
                m = vx;
        if (Math.abs(vy) > m)
            m = vy;
        vx = Math.abs(pow) * vx / Math.abs(m);
        vy = Math.abs(pow) * vy / Math.abs(m);
        drive(vx, vy, vy, vx);
        context.sleep(time);
        stop();
        context.telemetry.addData("status", "done");
        context.telemetry.update();
    }
    public void resetTicks() {
        encoderPos = BL.getCurrentPosition();
        hangEncoderPos = hang.getCurrentPosition();
        extendEncoderPos = extend.getCurrentPosition();
        liftEncoderPos = lift.getCurrentPosition();
    }
    public int getTicks() {
        return (int) ((BL.getCurrentPosition() - encoderPos) * RobotConstants.REV_ENCODER_COUNT / RobotConstants.NEVEREST_ENCODER_COUNT);
    }
    public int getLiftTicks() {
        return -(lift.getCurrentPosition() - liftEncoderPos);
    }
    public int getHangTicks() {
        return (hang.getCurrentPosition() - hangEncoderPos);
    }
    public int getExtendTicks() {
        return -(extend.getCurrentPosition() - extendEncoderPos);
    }
    public void nom(double power) {
        nom.setPower(power);
    }
    public void lift(double power) {
        lift.setPower(power);
    }
    public void hang(double power) {
        hang.setPower(power);
    }
    public void extend(double power) {
        extend.setPower(power);
    }
    public void nomRotator(double pos) {
        nomRotator.setPosition(pos);
    }
    public void placementRotator(double pos) {
        placementRotator.setPosition(pos);
    }
    public void markerServo(double pos) {
        markerServo.setPosition(pos);
    }
    public double nomRotatorPos() {
        return nomRotator.getPosition();
    }
    public double frontDist() {
        return frontDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public double backDist() {
        return backDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public double distDifference() {
        return frontDist() - backDist();
    }
    public void stop() {
        drive(0, 0, 0, 0);
    }
    double getKp() {
        return 0.02;
    }
}