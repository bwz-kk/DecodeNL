package org.firstinspires.ftc.teamcode.Commands.Vision;

import android.util.Log;

import androidx.annotation.NonNull;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.Vision.OdometryConstants;
import org.firstinspires.ftc.teamcode.Subsystem.Vision.VisionOdometry;

import java.util.Optional;

public class UpdatePoseCommand extends CommandBase {

    private static final String TAG = "UpdatePoseCommand";

    private final Follower follower;
    private final VisionOdometry vision;
    private final Pose fallbackPose;
    private boolean hasInitialized = false;

    public UpdatePoseCommand(
            @NonNull Follower follower,
            @NonNull VisionOdometry vision,
            @NonNull Pose fallbackPose
    ) {
        this.follower = follower;
        this.vision = vision;
        this.fallbackPose = fallbackPose;
        addRequirements();
    }

    public static void forceHardReset(
            @NonNull Follower follower,
            @NonNull VisionOdometry vision,
            double targetHeadingDegrees
    ) {
        double targetHeadingRad = Math.toRadians(targetHeadingDegrees);
        Pose currentPose = follower.getPose();
        follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), targetHeadingRad));

        vision.getRobotPoseMT2(targetHeadingRad).ifPresent(mt2Pose -> {
            follower.setPose(new Pose(
                    mt2Pose.getX(),
                    mt2Pose.getY(),
                    targetHeadingRad
            ));
            Log.i(TAG, "HARD RESET: Position updated 100% via Limelight");
        });
    }

    @Override
    public void initialize() {
        if (!hasInitialized) {
            Optional<Pose> initPoseMT2 = vision.getRobotPoseMT2(fallbackPose.getHeading());

            if (initPoseMT2.isPresent()) {
                Pose p = initPoseMT2.get();
                follower.setPose(new Pose(p.getX(), p.getY(), fallbackPose.getHeading()));
                Log.d(TAG, "Initialized via MT2 + Fallback Heading");
            } else {
                follower.setPose(fallbackPose);
                Log.w(TAG, "Camera blind at init. Using fallback pose.");
            }
            hasInitialized = true;
            return;
        }

        Pose currentPose = follower.getPose();

        vision.getRobotPoseMT2(currentPose.getHeading()).ifPresent(llPoseMT2 -> {
            double distInches = Math.hypot(
                    llPoseMT2.getX() - currentPose.getX(),
                    llPoseMT2.getY() - currentPose.getY()
            );
            double maxDeltaInches =
                    OdometryConstants.MAX_DELTA_METERS * OdometryConstants.METERS_TO_INCHES;

            if (distInches < maxDeltaInches) {
                Pose fusedPose = fusePoses(currentPose, llPoseMT2);
                follower.setPose(fusedPose);
            } else {
                Log.w(TAG, "MT2 Ignored: large jump (" + distInches + " in)");
            }
        });
    }

    @NonNull
    private Pose fusePoses(@NonNull Pose currentPose, @NonNull Pose llPose) {
        double wOdo = OdometryConstants.ODOMETRY_WEIGHT;
        double wLL = OdometryConstants.LIMELIGHT_WEIGHT;
        double total = wOdo + wLL;

        double fusedX = (currentPose.getX() * wOdo + llPose.getX() * wLL) / total;
        double fusedY = (currentPose.getY() * wOdo + llPose.getY() * wLL) / total;

        return new Pose(fusedX, fusedY, currentPose.getHeading());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public static void resetLocalizationStatus() {
    }
}