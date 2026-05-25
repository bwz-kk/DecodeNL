package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class TurretVision extends LimelightBase {

    private final KalmanFilter1D kalmanYaw;
    private final ElapsedTime updateTimer = new ElapsedTime();

    private double currentTargetYaw = 0.0;
    private double currentTargetArea = 0.0;
    private int currentTargetId = -1;

    public TurretVision() {
        kalmanYaw = new KalmanFilter1D(
                0,
                TurretVisionConstants.KALMAN_INITIAL_ERROR,
                TurretVisionConstants.KALMAN_PROCESS_NOISE,
                TurretVisionConstants.KALMAN_MEASUREMENT_NOISE
        );
    }

    public void init(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        init(
                hardwareMap,
                telemetry,
                TurretVisionConstants.HARDWARE_NAME,
                TurretVisionConstants.PIPELINE,
                TurretVisionConstants.DASHBOARD_STREAM_FPS
        );
        updateTimer.reset();
    }

    @Override
    public void update() {
        if (updateTimer.milliseconds() < TurretVisionConstants.INTERVAL_MS) {
            return;
        }
        updateTimer.reset();

        kalmanYaw.predict();

        LLResult result = limelight.getLatestResult();
        if (!isResultUsable(result)) {
            hasValidDetection = false;
            currentTargetId = -1;
            return;
        }

        processBestTarget(result);
    }

    private void processBestTarget(@NonNull LLResult result) {
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        if (tags == null || tags.isEmpty()) {
            hasValidDetection = false;
            currentTargetId = -1;
            return;
        }

        LLResultTypes.FiducialResult bestTag = selectBestTag(tags);

        if (bestTag == null) {
            hasValidDetection = false;
            currentTargetId = -1;
            return;
        }

        double rawYaw = bestTag.getTargetXDegrees();
        double filteredYaw = kalmanYaw.update(rawYaw);

        currentTargetYaw = filteredYaw;
        currentTargetArea = bestTag.getTargetArea();
        currentTargetId = bestTag.getFiducialId();
        hasValidDetection = true;
    }

    private LLResultTypes.FiducialResult selectBestTag(
            @NonNull List<LLResultTypes.FiducialResult> tags
    ) {
        LLResultTypes.FiducialResult bestTag = null;
        double bestArea = -1;

        for (LLResultTypes.FiducialResult tag : tags) {
            if (!isValidTagId(tag.getFiducialId(), TurretVisionConstants.VALID_TAG_IDS)) {
                continue;
            }
            double area = tag.getTargetArea();
            if (area > bestArea) {
                bestArea = area;
                bestTag = tag;
            }
        }

        return bestTag;
    }

    public double getTurretTargetYaw() {
        return hasValidDetection ? currentTargetYaw : 0.0;
    }

    public double getTargetArea() {
        return hasValidDetection ? currentTargetArea : 0.0;
    }

    public int getTargetId() {
        return hasValidDetection ? currentTargetId : -1;
    }

    @Deprecated
    public boolean hasTurretTarget() {
        return hasValidDetection;
    }

    public void resetTurretYaw(double value) {
        kalmanYaw.reset(value);
        currentTargetYaw = value;
    }
}