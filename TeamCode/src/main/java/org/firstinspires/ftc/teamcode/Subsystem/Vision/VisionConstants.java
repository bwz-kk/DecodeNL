package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import com.pedropathing.geometry.Pose;

public class VisionConstants {
    public static final String odometryLimelightName = "odometryLimelight";
    public static final int[] validTags = {21,24};
    public static final int limelightPipeline = 0;
    public static final double odometryCameraOffsetX = 0;
    public static final double odometryCameraOffsetY = 0;
    public static final double odometryCameraHeadingOffset = 0;
    public static final long intervalMS = 50; // 20 Hz

    // separação legal uou!!!
    public static final String turretLimelightName = "limelight-turret";
    public static final int[] validTurretTags = {21, 24};
    public static final int turretLimelightPipeline = 0;
    public static final long turretIntervalMS = 33; // ~30 Hz

    public static final double turretCameraOffsetX = 0;
    public static final double turretCameraOffsetY = 0;
    public static final double turretCameraHeadingOffset = 0;

    public static final Pose startPose = new Pose(0, 0, 0);

}
