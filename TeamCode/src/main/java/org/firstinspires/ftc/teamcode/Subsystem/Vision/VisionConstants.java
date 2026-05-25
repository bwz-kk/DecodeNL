package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import com.pedropathing.geometry.Pose;

public class VisionConstants {

    @Deprecated
    public static final String odometryLimelightName = OdometryConstants.HARDWARE_NAME;

    @Deprecated
    public static final int[] validTags = OdometryConstants.VALID_TAG_IDS;

    @Deprecated
    public static final int limelightPipeline = OdometryConstants.PIPELINE;

    @Deprecated
    public static final double odometryCameraOffsetX = OdometryConstants.CAMERA_OFFSET_X;

    @Deprecated
    public static final double odometryCameraOffsetY = OdometryConstants.CAMERA_OFFSET_Y;

    @Deprecated
    public static final double odometryCameraHeadingOffset = OdometryConstants.CAMERA_HEADING_OFFSET;

    @Deprecated
    public static final long intervalMS = OdometryConstants.INTERVAL_MS;

    @Deprecated
    public static final double ODOMETRY_WEIGHT = OdometryConstants.ODOMETRY_WEIGHT;

    @Deprecated
    public static final double LIMELIGHT_WEIGHT = OdometryConstants.LIMELIGHT_WEIGHT;

    @Deprecated
    public static final double MAX_DELTA_METERS = OdometryConstants.MAX_DELTA_METERS;

    @Deprecated
    public static final double METERS_TO_INCHES = OdometryConstants.METERS_TO_INCHES;

    @Deprecated
    public static final String turretLimelightName = TurretVisionConstants.HARDWARE_NAME;

    @Deprecated
    public static final int[] validTurretTags = TurretVisionConstants.VALID_TAG_IDS;

    @Deprecated
    public static final int turretLimelightPipeline = TurretVisionConstants.PIPELINE;

    @Deprecated
    public static final long turretIntervalMS = TurretVisionConstants.INTERVAL_MS;

    public static final Pose startPose = new Pose(0, 0, 0);
}