/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.misc.FtcUtils;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.chassis.IMU;


@Autonomous(name="Gyro Auto Singular Test", group = "Autonomous")
@Disabled
public class GyroAutoSingularTest extends LinearOpMode {
    private IMU imu = new IMU();
    double degs = 50; // degrees to turn
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "waiting for imu to init");
        telemetry.update();
        imu.init(hardwareMap, "imu");
        while (!imu.isGyroCalibrated() && opModeIsActive()) {
            idle();
        }
        imu.resetAngle();
        long startTime = System.currentTimeMillis();
        telemetry.addData("status", "waiting for start");
        telemetry.addData("globalAngle", imu.getAngle());
        telemetry.addData("goal", degs);
        telemetry.addData("global less than degs", FtcUtils.abs(imu.getAngle()) < FtcUtils.abs(degs));
        telemetry.update();
        waitForStart();
        degs = -degs;
        while (FtcUtils.abs(imu.getAngle()) < FtcUtils.abs(degs) && opModeIsActive()) {
            telemetry.addData("cur angle", imu.getAngle());
            telemetry.addData("angle diff", FtcUtils.abs(degs) - FtcUtils.abs(imu.getAngle()));
            telemetry.update();
            imu.updateAngle();
        }
        telemetry.addData("status", "done");
        telemetry.update();
    }
}
