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


@Autonomous(name="Depot Auto", group = "Autonomous")
public class DepotAuto extends LinearOpMode {
    private Robot robot = new Robot();
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "waiting for imu to init");
        telemetry.update();
        robot.init(hardwareMap, this, true, true);
        while (!robot.imu.isGyroCalibrated() && opModeIsActive()) {
            telemetry.addData("Status", "waiting for calibration");
            telemetry.update();
            idle();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.markerServo(RobotConstants.MARKERSERVO_HOLD);
        robot.deploy();
        if (robot.samplerTurnDegrees != 0) {
            sleep(100);
            if (robot.samplerTurnDegrees > 0) {
                robot.rotate(-(robot.samplerTurnDegrees + 95.0), .65, 3000);
                robot.moveTicks(-1400, .5, 3000);
                robot.moveTicks(50, .5, 500);
                robot.rotate(2.0 * robot.samplerTurnDegrees, .5, 3000);
                robot.moveTicks(-1150, .6, 3000);
                robot.rotate(50, .5, 3000);
                robot.dropTeamMarker();
                robot.rotate(46, .5, 3000);
            } else {
                robot.rotate(-(robot.samplerTurnDegrees + 90.0), .65, 3000);
                robot.moveTicks(-1300, .5, 3000);
                robot.moveTicks(50, .5, 500);
                robot.rotate(2.0 * robot.samplerTurnDegrees + 10, .6, 3000);
                robot.moveTicks(-1000, .6, 3000);
                robot.rotate(110, .5, 3000);
                robot.strafeTicks(200, .6, 500);
                robot.dropTeamMarker();
                robot.strafeTicks(-200, .6, 500);
                robot.rotate(45, .5, 3000);
            }
        } else {
            robot.rotate(-(robot.samplerTurnDegrees + 90.0), .65, 3000);
            sleep(600);
            robot.moveTicks(-1500, .6, 5000);
            sleep(600);
            robot.rotate(100.5, .5, 3000);
            sleep(200);
            robot.strafeTicks(300, .6, 1000);
            robot.dropTeamMarker();
            robot.strafeTicks(-150, 68, 1000);
            robot.rotate(37, .5, 3000);
        }
        robot.moveToCrater();
        sleep(200);
        robot.nomServo(RobotConstants.NOMSERVO_NEUTRAL);
    }
}
