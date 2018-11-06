package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.IMU;
import org.firstinspires.ftc.teamcode.misc.FtcUtils;
import org.firstinspires.ftc.teamcode.misc.RobotConstants;

@TeleOp(name = "Gyro Single Test")
@Disabled
public class GyroSingleTest extends LinearOpMode {
    private IMU imu = new IMU();
    private double rightx = 0;
    private double leftx = 0;
    private double lefty = 0;
    public void runOpMode() throws InterruptedException {
        imu.init(hardwareMap, "imu");
        telemetry.addData("Status", "Initialization bas been completed");
        telemetry.update();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            lefty = FtcUtils.motorScale(gamepad1.left_stick_y) * RobotConstants.sensitivity;
            rightx = FtcUtils.motorScale(gamepad1.right_stick_x) * RobotConstants.sensitivity;
            leftx = FtcUtils.motorScale(gamepad1.left_stick_x) * RobotConstants.sensitivity;
            telemetry.addData("gyro", imu.getAngle());
            imu.updateAngle();
            if (gamepad1.x) {
                imu.resetAngle();
            }
            telemetry.update();
            idle();
        }
    }
}