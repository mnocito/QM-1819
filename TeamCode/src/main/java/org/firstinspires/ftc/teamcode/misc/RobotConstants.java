package org.firstinspires.ftc.teamcode.misc;

public class RobotConstants {
    // max range for HS-7955TG is 750-2250uS, so .125-.875
    public static double NOMSERVO_UP = .7;
    public static double NOMSERVO_DOWN = .08;
    public static double NOMSERVO_NEUTRAL = .4;
    public static double MARKERSERVO_DROP = .7;
    public static double MARKERSERVO_HOLD = .35;
    public static double LOWEST_MOTOR_POWER = .2;
    public static double LOWEST_STRAFE_POWER = .4;
    public static double LOWEST_TURN_POWER = .24;
    public static double threshold = .5;
    public static double sensitivity = .9;
    public static double CATAPULT_MOTOR_POWER = .6;
    public static int MAX_EXTEND_TICKS = 8000;
    public static int MAX_HANG_TICKS = 15000;
    public static int LEFT_MAX_PIXEL_VALUE = 200;
    public static int RIGHT_MIN_PIXEL_VALUE= 400;
    public enum Position {
        LEFT, RIGHT, CENTER
    }
}