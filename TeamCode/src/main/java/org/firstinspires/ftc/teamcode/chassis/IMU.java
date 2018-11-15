package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.misc.FtcUtils;

public class IMU {
    private double globalAngle = 0;
    private double rotationAmt = 0;
    private double lastHeading = -1;
    private BNO055IMU imu;
    private double lastAngle;
    private double currentAngle;
    public void init(HardwareMap hwMap, String imuname) {
        BNO055IMU.Parameters parameters;
        imu = hwMap.get(BNO055IMU.class, imuname);
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        resetAngle();
    }
    public void resetAngle() {
        lastAngle = getRolledHeading();
        globalAngle = 0;
    }
    public double updateAngle() {
        currentAngle = getRolledHeading();
        globalAngle += currentAngle - lastAngle;
        lastAngle = currentAngle;
        return globalAngle;
    }
    private double getRawHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+180.0;
    }
    // create a heading that doesn't rollover at 360 or at 0
    public double getRolledHeading() {
        double heading = 360.0 - getRawHeading();
        if (lastHeading < 100 && heading > 300) {
            rotationAmt--;
        } else if (heading < 100 && lastHeading > 300) {
            rotationAmt++;
        }
        lastHeading = heading;
        return heading + rotationAmt * 360.0;
    }
    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }
    public double getAngle() {
        return globalAngle;
    }
}
