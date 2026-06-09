package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants;

public final class TurretVisionConstants {

    public static final String HARDWARE_NAME = "limelightTurret";
    public static final int PIPELINE = 0;
    public static final long INTERVAL_MS = 33;
    public static final int DASHBOARD_STREAM_FPS = 30;
    public static final double KALMAN_PROCESS_NOISE = 0.1;
    public static final double KALMAN_MEASUREMENT_NOISE = 2.0;
    public static final double KALMAN_INITIAL_ERROR = 50.0;
    public static final int[] VALID_TAG_IDS = {20, 24};
    public static final int BLUE_TAG_ID = 20;
    public static final int RED_TAG_ID  = 24;


    public static int getTagIdForSide(TurretConstants.SIDES side) {
        switch (side) {
            case RED:
                return RED_TAG_ID;
            case BLUE:
            default:
                return BLUE_TAG_ID;
        }
    }
}