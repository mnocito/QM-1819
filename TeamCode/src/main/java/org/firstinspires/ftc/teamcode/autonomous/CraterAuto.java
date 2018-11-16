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

import org.firstinspires.ftc.teamcode.chassis.Robot;
import org.firstinspires.ftc.teamcode.misc.FtcUtils;
import org.firstinspires.ftc.teamcode.misc.RobotConstants;


@Autonomous(name="Crater Auto", group = "Autonomous")
public class CraterAuto extends LinearOpMode {
    private Robot robot = new Robot();
    private double samplerTurnDegrees = 0;
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "waiting for imu to init");
        telemetry.update();
        robot.init(hardwareMap, this, true, true);
        while (!robot.imu.isGyroCalibrated() && opModeIsActive()) {
            telemetry.addData("Status", "waiting for calibration");
            telemetry.update();
            idle();
        }
        robot.markerServo(RobotConstants.MARKERSERVO_HOLD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        if (robot.canSample) samplerTurnDegrees = robot.getSamplerTurnDegrees(2500);
        robot.deploy();
        robot.rotate(-(samplerTurnDegrees + 90.0), .5, 3000);
        if (samplerTurnDegrees != 0) {
            sleep(250);
            robot.moveTicks(-850, .5, 3000);
            sleep(250);
            robot.moveTicks(400, .5, 5000);
            if (samplerTurnDegrees == 30) {
                robot.rotate(-60.0, .5, 3000);
                sleep(200);
                robot.moveTicks(2000, .5, 5000);
            } else {
                robot.rotate(-120.0, .5, 3000);
                sleep(200);
                robot.moveTicks(2400, .5, 5000);
            }
            sleep(200);
            robot.rotate(-48, .5, 3000);
            robot.strafeTicks(600, .5, 3000);
            robot.strafeTicks(-200, .5, 3000);
            sleep(200);
            robot.moveTicks(1450, .5, 5000);
            sleep(200);
            robot.dropTeamMarker();
            sleep(200);
            robot.moveTicks(-2650, .6, 5000);
        } else {
            sleep(250);
            robot.moveTicks(-650, .5, 5000);
            robot.moveTicks(400, .5, 5000);
            sleep(250);
            robot.rotate(90, .5, 3000);
            sleep(200);
            robot.moveTicks(2200, .5, 5000);
            sleep(200);
            robot.rotate(-48, .5, 3000);
            robot.strafeTicks(600, .5, 3000);
            robot.strafeTicks(-200, .5, 3000);
            sleep(200);
            robot.moveTicks(1450, .5, 5000);
            sleep(200);
            robot.dropTeamMarker();
            sleep(200);
            robot.moveTicks(-2650, .6, 5000);
        }

        robot.nomServo(RobotConstants.NOMSERVO_NEUTRAL);
        sleep(2000);
    }
}
