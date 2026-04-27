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
    private Limelight3A limelight;
    private Telemetry telemetry;

    private Pose currentVisionPose = null;
    private boolean hasValidDetection = false;

    private final ElapsedTime updateTimer = new ElapsedTime();

    // -------------------------------------------------------------------------
    // Inicialização
    // -------------------------------------------------------------------------

    /**
     * Inicializa e configura a Limelight 3A.
     *
     * @param hardwareMap mapa de hardware do OpMode
     * @param telemetry   telemetria para debug em campo
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);

        limelight.pipelineSwitch(VisionConstants.APRILTAG_PIPELINE);
        limelight.start();

        updateTimer.reset();

        telemetry.addLine("[Vision] Limelight inicializada.");
    }

    // -------------------------------------------------------------------------
    // Loop principal
    // -------------------------------------------------------------------------

    /**
     * Deve ser chamado em cada iteração do loop do OpMode.
     * Respeita o intervalo mínimo definido em VisionConstants.UPDATE_INTERVAL_MS.
     */
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (!isResultUsable(result)) {
            hasValidDetection = false;
            currentVisionPose = null;
            return;
        }

        processBestTag(result);
    }

    // -------------------------------------------------------------------------
    // Processamento de tags
    // -------------------------------------------------------------------------

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

        currentVisionPose = convertAndCompensate(rawPose);
        hasValidDetection = true;
    }

    private LLResultTypes.FiducialResult selectBestTag(List<LLResultTypes.FiducialResult> tags) {
        LLResultTypes.FiducialResult best = null;
        double bestDistance = Double.MAX_VALUE;

        for (LLResultTypes.FiducialResult tag : tags) {
            if (!isValidTagId(tag.getFiducialId())) continue;
        }

        return best;
    }

    private Pose convertAndCompensate(Pose3D rawPose) {
        final double METERS_TO_INCHES = 39.3701;

        // Pose bruta em polegadas
        double x = rawPose.getPosition().x * METERS_TO_INCHES;
        double y = rawPose.getPosition().y * METERS_TO_INCHES;

        // Heading: Limelight usa convenção WPILib (CCW positivo, radianos)
        // PedroPathing também usa radianos CCW — sem conversão necessária
        double heading = rawPose.getOrientation().getYaw();

        return new Pose(x, y, heading);
    }

    public boolean resetPoseFromTag(Follower follower) {
        if (!hasValidDetection || currentVisionPose == null) {
            telemetry.addLine("[Vision] Reset ignorado — sem tag válida.");
            return false;
        }

        follower.setPose(currentVisionPose);

        telemetry.addData("[Vision] Pose resetada via AprilTag",
                String.format("X=%.1f  Y=%.1f  H=%.2f°",
                        currentVisionPose.getX(),
                        currentVisionPose.getY(),
                        Math.toDegrees(currentVisionPose.getHeading())));

        return true;
    }

    public boolean hasValidTag() {
        return hasValidDetection && currentVisionPose != null;
    }

    /** pose estimada pela câmera */
    public Pose getCurrentVisionPose() {
        return currentVisionPose;
    }

    /** Para o streaming da Limelight. Chame em stop() do OpMode. */
    public void stop() {
        if (limelight != null) limelight.stop();
    }

    private boolean isResultUsable(LLResult result) {
        if (result == null) return false;

        LLStatus status = limelight.getStatus();
        // pipelineIndex garante que estamos no pipeline correto
        return status != null && result.isValid();
    }

    private boolean isValidTagId(int id) {
        for (int valid : VisionConstants.VALID_TAG_IDS) {
            if (valid == id) return true;
        }
        return false;
    }

    private double getTagDistance(LLResultTypes.FiducialResult tag) {
        Pose3D pose = tag.getRobotPoseFieldSpace();
        if (pose == null) return Double.MAX_VALUE;

        double x = pose.getPosition().x;
        double y = pose.getPosition().y;
        double z = pose.getPosition().z;

        return Math.sqrt(x*x + y*y + z*z) * 39.3701; // metros → polegadas
    }
}




