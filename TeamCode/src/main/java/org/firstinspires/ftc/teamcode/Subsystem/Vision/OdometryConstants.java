package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class OdometryConstants {

    public static final String HARDWARE_NAME = "odometryLimelight";
    public static final int PIPELINE = 0;
    public static final long INTERVAL_MS = 50;
    public static final double CAMERA_OFFSET_X = 0;
    public static final double CAMERA_OFFSET_Y = 6;
    public static final double CAMERA_HEADING_OFFSET = 0;
    public static final double ODOMETRY_WEIGHT = 0.7;
    public static final double LIMELIGHT_WEIGHT = 0.3;
    public static final double MAX_DELTA_METERS = 1.0;
    public static final double METERS_TO_INCHES = 39.3701;
    public static final double KALMAN_PROCESS_NOISE = 0.1;
    public static final double KALMAN_MEASUREMENT_NOISE = 1.0;
    public static final double KALMAN_INITIAL_ERROR = 100.0;
    public static final int[] VALID_TAG_IDS = {20, 24};
    public static final double X_SIGN_FLIP = -1.0;

}