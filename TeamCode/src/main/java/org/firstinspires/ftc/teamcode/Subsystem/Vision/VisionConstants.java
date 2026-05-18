package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import com.pedropathing.geometry.Pose;

public class VisionConstants {
    public static final String odometryLimelightName = "odometryLimelight";
    public static final int[] validTags = {21,24};
    public static final int limelightPipeline = 0;
    public static final double odometryCameraOffsetX = 0;
    public static final double odometryCameraOffsetY = 6;
    public static final double odometryCameraHeadingOffset = 0;
    public static final long intervalMS = 33; // 20 Hz
    public static final double ODOMETRY_WEIGHT = 0.7;      // Trust odometry more (0.8-0.9)
    public static final double LIMELIGHT_WEIGHT = 0.3;     // Trust vision less (0.1-0.2)
    public static final double MAX_DELTA_METERS = 1;     // Stricter outlier rejection

    // separação legal uou!!!
    public static final String turretLimelightName = "limelight-turret";
    public static final int[] validTurretTags = {21, 24};
    public static final int turretLimelightPipeline = 0;
    public static final long turretIntervalMS = 33; // ~30 Hz
    public static final Pose startPose = new Pose(0, 0, 0);

}
