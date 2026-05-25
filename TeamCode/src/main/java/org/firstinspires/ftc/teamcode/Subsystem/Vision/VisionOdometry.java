package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.Optional;

public class VisionOdometry extends LimelightBase {

    private final KalmanFilter1D kalmanX;
    private final KalmanFilter1D kalmanY;
    private final KalmanFilter1D kalmanH;
    private final ElapsedTime updateTimer = new ElapsedTime();

    private Pose currentVisionPose = null;

    public VisionOdometry() {
        kalmanX = new KalmanFilter1D(
                0,
                OdometryConstants.KALMAN_INITIAL_ERROR,
                OdometryConstants.KALMAN_PROCESS_NOISE,
                OdometryConstants.KALMAN_MEASUREMENT_NOISE
        );
        kalmanY = new KalmanFilter1D(
                0,
                OdometryConstants.KALMAN_INITIAL_ERROR,
                OdometryConstants.KALMAN_PROCESS_NOISE,
                OdometryConstants.KALMAN_MEASUREMENT_NOISE
        );
        kalmanH = new KalmanFilter1D(
                0,
                OdometryConstants.KALMAN_INITIAL_ERROR,
                OdometryConstants.KALMAN_PROCESS_NOISE,
                OdometryConstants.KALMAN_MEASUREMENT_NOISE
        );
    }

    public void init(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        init(
                hardwareMap,
                telemetry,
                OdometryConstants.HARDWARE_NAME,
                OdometryConstants.PIPELINE,
                30
        );
        updateTimer.reset();
    }

    @Override
    public void update() {
        if (updateTimer.milliseconds() < OdometryConstants.INTERVAL_MS) {
            return;
        }
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

    private void processBestTag(@NonNull LLResult result) {
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
            currentVisionPose = null;
            return;
        }

        Pose compensated = convertAndCompensate(rawPose);

        double filteredX = kalmanX.update(compensated.getX());
        double filteredY = kalmanY.update(compensated.getY());
        double filteredH = kalmanH.update(compensated.getHeading());

        currentVisionPose = new Pose(filteredX, filteredY, filteredH);
        hasValidDetection = true;
    }

    public Optional<Pose> getRobotPoseMT2(double headingRadians) {
        LLResult result = limelight.getLatestResult();
        if (!isResultUsable(result)) {
            return Optional.empty();
        }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) {
            return Optional.empty();
        }

        double x = botpose.getPosition().x * OdometryConstants.METERS_TO_INCHES
                - OdometryConstants.CAMERA_OFFSET_X;
        double y = botpose.getPosition().y * OdometryConstants.METERS_TO_INCHES
                - OdometryConstants.CAMERA_OFFSET_Y;

        return Optional.of(new Pose(x, y, headingRadians));
    }

    private LLResultTypes.FiducialResult selectBestTag(
            @NonNull List<LLResultTypes.FiducialResult> tags
    ) {
        LLResultTypes.FiducialResult bestTag = null;
        double bestArea = -1;

        for (LLResultTypes.FiducialResult tag : tags) {
            if (!isValidTagId(tag.getFiducialId(), OdometryConstants.VALID_TAG_IDS)) {
                continue;
            }
            Pose3D pose = tag.getRobotPoseFieldSpace();
            if (pose == null) {
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

    private Pose convertAndCompensate(@NonNull Pose3D rawPose) {
        double x = rawPose.getPosition().x * OdometryConstants.METERS_TO_INCHES;
        double y = rawPose.getPosition().y * OdometryConstants.METERS_TO_INCHES;
        double heading = rawPose.getOrientation().getYaw();

        x -= OdometryConstants.CAMERA_OFFSET_X * Math.cos(heading)
                - OdometryConstants.CAMERA_OFFSET_Y * Math.sin(heading);
        y -= OdometryConstants.CAMERA_OFFSET_X * Math.sin(heading)
                + OdometryConstants.CAMERA_OFFSET_Y * Math.cos(heading);

        heading -= OdometryConstants.CAMERA_HEADING_OFFSET;

        return new Pose(x, y, heading);
    }

    public boolean resetPoseFromTag(@NonNull Follower follower) {
        if (!hasValidDetection || currentVisionPose == null) {
            if (telemetry != null) {
                telemetry.addLine("Vision: No valid detection for reset");
            }
            return false;
        }

        Pose currentPose = follower.getPose();
        Pose visionPose = currentVisionPose;

        double distInches = Math.hypot(
                visionPose.getX() - currentPose.getX(),
                visionPose.getY() - currentPose.getY()
        );
        double maxDeltaInches =
                OdometryConstants.MAX_DELTA_METERS * OdometryConstants.METERS_TO_INCHES;

        if (distInches > maxDeltaInches) {
            if (telemetry != null) {
                telemetry.addLine("Vision: Outlier rejected (" + distInches + " in)");
            }
            return false;
        }

        double wOdo = OdometryConstants.ODOMETRY_WEIGHT;
        double wLL = OdometryConstants.LIMELIGHT_WEIGHT;
        double total = wOdo + wLL;

        double fusedX = (currentPose.getX() * wOdo + visionPose.getX() * wLL) / total;
        double fusedY = (currentPose.getY() * wOdo + visionPose.getY() * wLL) / total;

        Pose fusedPose = new Pose(fusedX, fusedY, currentPose.getHeading());

        follower.setPose(fusedPose);

        kalmanX.reset(fusedX);
        kalmanY.reset(fusedY);
        kalmanH.reset(currentPose.getHeading());

        if (telemetry != null) {
            telemetry.addLine("Vision: Pose corrected (fused)");
        }
        return true;
    }

    @Deprecated
    public boolean hasValidTag() {
        return hasValidDetection && currentVisionPose != null;
    }

    @Nullable
    public Pose getCurrentVisionPose() {
        return currentVisionPose;
    }
}