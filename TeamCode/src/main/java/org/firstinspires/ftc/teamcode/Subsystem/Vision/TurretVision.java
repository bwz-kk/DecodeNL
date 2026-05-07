package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class TurretVision {
    private final KalmanFilter1D kalmanYaw;
    private Limelight3A turretLimelight;
    private Telemetry telemetry;
    private double currentTargetYaw = 0.0;
    private double currentTargetArea = 0.0;
    private int currentTargetId = -1;
    private boolean hasValidTarget = false;

    private final ElapsedTime updateTimer = new ElapsedTime();

    public TurretVision() {
        kalmanYaw = new KalmanFilter1D(0, 50,0,0);
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        turretLimelight = hardwareMap.get(Limelight3A.class, VisionConstants.turretLimelightName);
        turretLimelight.pipelineSwitch(VisionConstants.turretLimelightPipeline);
        turretLimelight.start();
        FtcDashboard.getInstance().startCameraStream(turretLimelight, 120);

        updateTimer.reset();
    }

    public void update() {
        if (updateTimer.milliseconds() < VisionConstants.turretIntervalMS) {
            return;
        }
        updateTimer.reset();
        kalmanYaw.predict();

        LLResult result = turretLimelight.getLatestResult();

        if (!isResultUsable(result)) {
            hasValidTarget = false;
            currentTargetId = -1;
            return;
        }

        processBestTarget(result);
    }

    private void processBestTarget(LLResult result) {
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        if (tags == null || tags.isEmpty()) {
            hasValidTarget = false;
            currentTargetId = -1;
            return;
        }

        LLResultTypes.FiducialResult bestTag = selectBestTag(tags);

        if (bestTag == null) {
            hasValidTarget = false;
            currentTargetId = -1;
            return;
        }

        // Extract raw measurements
        double rawYaw = bestTag.getTargetXDegrees();
        double targetArea = bestTag.getTargetArea();
        int tagId = bestTag.getFiducialId();

        // Update Kalman filter with raw yaw measurement
        double filteredYaw = kalmanYaw.update(rawYaw);

        currentTargetYaw = filteredYaw;
        currentTargetArea = targetArea;
        currentTargetId = tagId;
        hasValidTarget = true;
    }

    private LLResultTypes.FiducialResult selectBestTag(List<LLResultTypes.FiducialResult> tags) {
        LLResultTypes.FiducialResult bestTag = null;
        double bestArea = -1;

        for (LLResultTypes.FiducialResult tag : tags) {
            if (!isValidTagId(tag.getFiducialId())) continue;

            double area = tag.getTargetArea();
            if (area > bestArea) {
                bestArea = area;
                bestTag = tag;
            }
        }

        return bestTag;
    }

    public double getTurretTargetYaw() {
        return hasValidTarget ? currentTargetYaw : 0.0;
    }

    public double getTargetArea() {
        return hasValidTarget ? currentTargetArea : 0.0;
    }

    public int getTargetId() {
        return hasValidTarget ? currentTargetId : -1;
    }

    public boolean hasTurretTarget() {
        return hasValidTarget;
    }

    public void resetTurretYaw(double value) {
        kalmanYaw.reset(value);
        currentTargetYaw = value;
    }

    public void stop() {
        if (turretLimelight != null) {
            turretLimelight.stop();
        }
    }

    private boolean isResultUsable(LLResult result) {
        if (result == null) return false;
        LLStatus status = turretLimelight.getStatus();
        return status != null && result.isValid();
    }

    private boolean isValidTagId(int id) {
        for (int valid : VisionConstants.validTurretTags) {
            if (valid == id) return true;
        }
        return false;
    }
}
