package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU {
    private Orientation lastAngles;
    private Orientation currentAngles;
    private double globalAngle = 0;
    private BNO055IMU imu;
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
        lastAngles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    public double updateAngle() {
        currentAngles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle += FtcUtils.normalizeDegrees(currentAngles.firstAngle - lastAngles.firstAngle);
        lastAngles = currentAngles;
        return globalAngle;
    }
    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }
    public double getAngle() {
        return globalAngle;
    }

}
