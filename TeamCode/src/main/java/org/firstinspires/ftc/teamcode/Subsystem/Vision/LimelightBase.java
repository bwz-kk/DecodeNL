package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class LimelightBase {

    protected Limelight3A limelight;
    protected Telemetry telemetry;
    protected boolean hasValidDetection = false;

    protected void init(
            @NonNull HardwareMap hardwareMap,
            @Nullable Telemetry telemetry,
            @NonNull String hardwareName,
            int pipeline,
            int dashboardFps
    ) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, hardwareName);
        limelight.pipelineSwitch(pipeline);
        startCamera();
        if (dashboardFps > 0) {
            startDashboardStream(dashboardFps);
        }
    }

    protected void startCamera() {
        if (limelight != null) {
            limelight.start();
        }
    }

    protected void startDashboardStream(int fps) {
        FtcDashboard.getInstance().startCameraStream(limelight, fps);
    }

    public abstract void update();

    protected boolean isResultUsable(@Nullable LLResult result) {
        if (result == null) return false;
        if (limelight == null) return false;
        LLStatus status = limelight.getStatus();
        return status != null && result.isValid();
    }

    protected boolean isValidTagId(int id, @NonNull int[] validIds) {
        for (int valid : validIds) {
            if (valid == id) return true;
        }
        return false;
    }

    public boolean hasValidDetection() {
        return hasValidDetection;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}