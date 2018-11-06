package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.misc.FtcUtils;
import org.firstinspires.ftc.teamcode.misc.RobotConstants;

import java.util.List;

public class Sampler {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private HardwareMap hwMap;
    private LinearOpMode context;
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AUbqP2X/////AAABmanjyi8qNEe4lL7GfntMCVJ562TM830n+NlX1vnENsuMEzeIsEDF/qyeNwM4x+DziWQsfsvpp0SxW5eKQPYRjN7Ie5rdVCCE//IJGv15r5yrTflq/EBo8JRb6VT97RINry7r0rXsCsm6OaB4CgiboJMt0sWuPLL4mCEgREvCSAOjaHcu5vIklq1fS4i8F89EyPm6J/ctw1ePPGUaZyT7vqnGb1P3gF+oELUbkXG/ui3PAeuOXuhbe2eJ0CATQWjV5P4C4v1cL4nhcYjA0Npe/KLQplbQSr4iD141V7dX8rWNYltQj3oSg2TyrMR6bOse6H+06lRfCB/s83pd19/U4gur0LYYJI5lHHVRQh9/n87d";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public void init(HardwareMap hwMap, LinearOpMode context) {
        this.hwMap = hwMap;
        this.context = context;
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            context.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            context.telemetry.update();
        }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public RobotConstants.Position getPosition(int timeout) {
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        if (tfod != null) {
            tfod.activate();
        } else {
            context.telemetry.addData("status", "tensorflow not enabled");
            context.telemetry.update();
            return RobotConstants.Position.CENTER;
        }
        while (context.opModeIsActive() && currentTime - startTime < timeout) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    context.telemetry.addData("# Object Detected", updatedRecognitions.size());
                    context.telemetry.update();
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                if (tfod != null) tfod.shutdown();
                                context.telemetry.addData("Gold Mineral Position", RobotConstants.Position.LEFT);
                                context.telemetry.update();
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                if (tfod != null) tfod.shutdown();
                                context.telemetry.addData("Gold Mineral Position", RobotConstants.Position.RIGHT);
                                context.telemetry.update();
                            } else {
                                if (tfod != null) tfod.shutdown();
                                context.telemetry.addData("Gold Mineral Position", RobotConstants.Position.CENTER);
                                context.telemetry.update();
                                return RobotConstants.Position.CENTER;
                            }
                        }
                    }
                }
            }
            currentTime = System.currentTimeMillis();
        }
        return RobotConstants.Position.CENTER;
    }
}
