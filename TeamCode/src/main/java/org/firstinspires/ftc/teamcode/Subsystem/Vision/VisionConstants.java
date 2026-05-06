package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import com.pedropathing.geometry.Pose;

public class VisionConstants {
    public static final String limelightName = "limelight";
    public static final int[] validTags = {21,24};
    public static final int limelightPipeline = 0;
    public static final Pose startPose = new Pose(0, 0, 0);
    public static final double cameraOffsetX= 0;
    public static final double cameraOffsetY= 0;
    public static final double cameraHeadingOffset = 0;
    public static final long intervalMS = 50; // 20 Hz

}
