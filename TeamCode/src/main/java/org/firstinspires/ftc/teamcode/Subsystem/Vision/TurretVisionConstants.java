package org.firstinspires.ftc.teamcode.Subsystem.Vision;

public final class TurretVisionConstants {

    public static final String HARDWARE_NAME = "limelight-turret";
    public static final int PIPELINE = 0;
    public static final long INTERVAL_MS = 33;
    public static final int DASHBOARD_STREAM_FPS = 120;
    public static final double KALMAN_PROCESS_NOISE = 0.1;
    public static final double KALMAN_MEASUREMENT_NOISE = 2.0;
    public static final double KALMAN_INITIAL_ERROR = 50.0;
    public static final int[] VALID_TAG_IDS = {20, 24};

    private TurretVisionConstants() {}
}