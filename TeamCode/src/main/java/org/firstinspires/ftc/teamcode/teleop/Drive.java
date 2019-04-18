package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.chassis.Robot;
import org.firstinspires.ftc.teamcode.misc.FtcUtils;
import org.firstinspires.ftc.teamcode.misc.RobotConstants;

@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    private double currentNomServoPos = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private double rightx = 0;
    private double leftx = 0;
    private double lefty = 0;
    double FLPow = 0;
    double BLPow = 0;
    double FRPow = 0;
    double BRPow = 0;
    double[] pows = new double[4];
    double m = 0.0;
    private boolean pressed = false;
    private Robot robot = new Robot();
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this, false);
        //robot.markerServo(RobotConstants.MARKERSERVO_HOLD);
        telemetry.addData("Status", "Initialization bas been completed");
        telemetry.update();
        waitForStart();
        robot.placementRotator(RobotConstants.PLACEMENTSERVO_RECEIVE);
        while (!isStopRequested() && opModeIsActive()) {
            m = 0;
            telemetry.addData("placement position", robot.placementRotatorPos());
            telemetry.addData("nom position", robot.nomRotatorPos());
            //telemetry.addData("lift", robot.getLiftTicks());
            //telemetry.addData("hang ticks", robot.getHangTicks());
            lefty = FtcUtils.motorScale(-gamepad1.left_stick_y) * RobotConstants.sensitivity;
            lefty = FtcUtils.abs(lefty) >= RobotConstants.threshold ? lefty : 0;
            rightx = FtcUtils.motorScale(gamepad1.right_stick_x) * RobotConstants.sensitivity;
            rightx = FtcUtils.abs(rightx) >= RobotConstants.threshold ? rightx : 0;
            leftx = FtcUtils.motorScale(gamepad1.left_stick_x) * RobotConstants.sensitivity;
            leftx = FtcUtils.abs(leftx) >= RobotConstants.threshold ? leftx : 0;
            //      Front Left = +Speed + Turn - Strafe      Front Right  = +Speed - Turn + Strafe
            //      Back Left  = +Speed + Turn + Strafe      Back Right  = +Speed - Turn - Strafe
            pows[0] = lefty + leftx + rightx;
            pows[1] = lefty - leftx + rightx;
            pows[2] = lefty - leftx - rightx;
            pows[3] = lefty + leftx - rightx;
            for (double v : pows)
                if (Math.abs(v) > m)
                    m = v;
            if (m != 0) { // if the max power isn't 0 (can't divide by 0)
                pows[0] /= Math.abs(m);
                pows[1] /= Math.abs(m);
                pows[2] /= Math.abs(m);
                pows[3] /= Math.abs(m);
            }
            pows[0] = FtcUtils.abs(pows[0]) > RobotConstants.threshold ? pows[0] : 0;
            pows[1] = FtcUtils.abs(pows[1]) > RobotConstants.threshold ? pows[1] : 0;
            pows[2] = FtcUtils.abs(pows[2]) > RobotConstants.threshold ? pows[2] : 0;
            pows[3] = FtcUtils.abs(pows[3]) > RobotConstants.threshold ? pows[3] : 0;
            robot.drive(FtcUtils.motorScale(pows[0]), FtcUtils.motorScale(pows[1]), FtcUtils.motorScale(pows[2]), FtcUtils.motorScale(pows[3]));
            if (FtcUtils.motorScale(Math.sqrt(gamepad2.right_trigger)) > RobotConstants.threshold) {
                robot.nom(FtcUtils.motorScale(Math.sqrt(gamepad2.right_trigger)));
            } else if (FtcUtils.motorScale(Math.sqrt(gamepad2.left_trigger)) > RobotConstants.threshold) {
                robot.nom(FtcUtils.motorScale(-Math.sqrt(gamepad2.left_trigger)));
            } else {
                robot.nom(0);
            }
            if (FtcUtils.motorScale(Math.sqrt(gamepad1.right_trigger)) > RobotConstants.threshold) {
                robot.extend(FtcUtils.motorScale(-Math.sqrt(gamepad1.right_trigger)));
            } else if (FtcUtils.motorScale(Math.sqrt(gamepad1.left_trigger)) > RobotConstants.threshold) {
                robot.extend(FtcUtils.motorScale(Math.sqrt(gamepad1.left_trigger)));
            } else if (gamepad1.left_bumper) {
                robot.extend(.35);
            } else if (gamepad1.right_bumper) {
                robot.extend(-.35);
            } else {
                robot.extend(0);
            }
           /* if (FtcUtils.abs(FtcUtils.motorScale(gamepad2.left_stick_y)) > RobotConstants.threshold) {
               // robot.extend(FtcUtils.motorScale(gamepad2.left_stick_y));
                if (robot.canExtendOut()) {
                    robot.extend(FtcUtils.motorScale(gamepad2.left_stick_y));
                } else if (robot.canExtendIn()) {
                    robot.extend(FtcUtils.motorScale(gamepad2.left_stick_y));
                }
            } else {
                robot.extend(0);
            }*/
            telemetry.addData("gamepad pow", FtcUtils.sign(gamepad2.left_stick_y)*FtcUtils.motorScale(gamepad2.left_stick_y)*FtcUtils.motorScale(gamepad2.left_stick_y));
            telemetry.addData("lift power", robot.liftPower());
            if (FtcUtils.abs(FtcUtils.motorScale(gamepad2.left_stick_y)*FtcUtils.motorScale(gamepad2.left_stick_y)) > RobotConstants.threshold && robot.canLift()) {
                if (robot.canLiftUp()) {
                    robot.lift(FtcUtils.sign(gamepad2.left_stick_y)*FtcUtils.motorScale(gamepad2.left_stick_y)*FtcUtils.motorScale(gamepad2.left_stick_y));
                } else if (/*robot.canLiftDown() && */FtcUtils.abs(FtcUtils.motorScale(gamepad2.left_stick_y)*FtcUtils.motorScale(gamepad2.left_stick_y) * .5) > RobotConstants.threshold) {
                    robot.lift(FtcUtils.sign(gamepad2.left_stick_y)*FtcUtils.motorScale(gamepad2.left_stick_y)*FtcUtils.motorScale(gamepad2.left_stick_y) * .5);
                }
            } else {
                robot.lift(0);
            }
            if (gamepad2.y) {
                robot.nomRotator(RobotConstants.NOMSERVO_UP);
            }
            if (gamepad2.x) {
                robot.nomRotator(RobotConstants.NOMSERVO_DOWN);
            }
            if (gamepad2.b) {
                robot.placementRotator(RobotConstants.PLACEMENTSERVO_PLACE);
            }
            if (gamepad2.a) {
                robot.placementRotator(RobotConstants.PLACEMENTSERVO_RECEIVE);
            }
            if (gamepad2.right_bumper) {
                robot.nomRotator(RobotConstants.NOMSERVO_NEUTRAL);
            }
            if (gamepad2.dpad_up) {
                robot.hang(1);
            } else if (gamepad2.dpad_down) {
                robot.hang(-1);
            } else {
                robot.hang(0);
            }
            telemetry.update();
            idle();
        }
        robot.stop();
    }
}