package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class VisionOdometry {
    // -------------------------------------------------------------------------
    // Kalman filters — one per axis
    // -------------------------------------------------------------------------

    private final KalmanFilter1D kalmanX;
    private final KalmanFilter1D kalmanY;
    private final KalmanFilter1D kalmanH;
    private Limelight3A limelight;
    private Telemetry telemetry;
    private Pose currentVisionPose = null;
    private boolean hasValidDetection = false;

    private final ElapsedTime updateTimer = new ElapsedTime();

    public VisionOdometry() {
        kalmanX = new KalmanFilter1D(0, 100,0,0);
        kalmanY = new KalmanFilter1D(0, 100,0,0);
        kalmanH = new KalmanFilter1D(0, 100,0,0);
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
        limelight.pipelineSwitch(VisionConstants.limelightPipeline);
        limelight.start();

        updateTimer.reset();
    }

    public void update() {
        if (updateTimer.milliseconds() < VisionConstants.intervalMS) return;
        updateTimer.reset();

        kalmanX.predict();
        kalmanY.predict();
        kalmanH.predict();

        LLResult result = limelight.getLatestResult();

        if (!isResultUsable(result)) {
            hasValidDetection = false;
            currentVisionPose = null;
            return;
        }

        processBestTag(result);
    }

    private void processBestTag(LLResult result) {
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        if (tags == null || tags.isEmpty()) {
            hasValidDetection = false;
            currentVisionPose = null;
            return;
        }

        LLResultTypes.FiducialResult bestTag = selectBestTag(tags);

        if (bestTag == null) {
            hasValidDetection = false;
            currentVisionPose = null;
            return;
        }

        Pose3D rawPose = bestTag.getRobotPoseFieldSpace();

        if (rawPose == null) {
            hasValidDetection = false;
            return;
        }

        Pose compensated = convertAndCompensate(rawPose);

        double filteredX = kalmanX.update(compensated.getX());
        double filteredY = kalmanY.update(compensated.getY());
        double filteredH = kalmanH.update(compensated.getHeading());

        currentVisionPose = new Pose(filteredX, filteredY, filteredH);
        hasValidDetection = true;
    }

    private LLResultTypes.FiducialResult selectBestTag(List<LLResultTypes.FiducialResult> tags) {
        for (LLResultTypes.FiducialResult tag : tags) {
            if (!isValidTagId(tag.getFiducialId())) continue;
            if (tag.getRobotPoseFieldSpace() == null) continue;


            return tag;
        }

        return null;
    }

    private Pose convertAndCompensate(Pose3D rawPose) {
        final double MetersToInches = 39.3701;

        double x = rawPose.getPosition().x * MetersToInches;
        double y = rawPose.getPosition().y * MetersToInches;
        double heading = rawPose.getOrientation().getYaw();

        x -= VisionConstants.cameraOffsetX * Math.cos(heading) - VisionConstants.cameraOffsetY * Math.sin(heading);
        y -= VisionConstants.cameraOffsetX * Math.sin(heading) + VisionConstants.cameraOffsetY * Math.cos(heading);

        heading -= VisionConstants.cameraHeadingOffset;

        return new Pose(x, y, heading);
    }
    public boolean resetPoseFromTag(Follower follower) {
        if (!hasValidDetection || currentVisionPose == null) {
            return false;
        }

        follower.setPose(currentVisionPose);

        kalmanX.reset(currentVisionPose.getX());
        kalmanY.reset(currentVisionPose.getY());
        kalmanH.reset(currentVisionPose.getHeading());

        return true;
    }

    public boolean hasValidTag() {
        return hasValidDetection && currentVisionPose != null;
    }

    public Pose getCurrentVisionPose() {
        return currentVisionPose;
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    private boolean isResultUsable(LLResult result) {
        if (result == null) return false;
        LLStatus status = limelight.getStatus();
        return status != null && result.isValid();
    }

    private boolean isValidTagId(int id) {
        for (int valid : VisionConstants.validTags) {
            if (valid == id) return true;
        }
        return false;
    }

    private double getTagDistance(LLResultTypes.FiducialResult tag) {
        double ta = tag.getTargetArea();
        return ta > 0 ? (1.0 / ta) * 10.0 : Double.MAX_VALUE;
    }
}





