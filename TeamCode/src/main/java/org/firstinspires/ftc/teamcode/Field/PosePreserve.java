package org.firstinspires.ftc.teamcode.Field;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants;

public class PosePreserve {
    public static Turret turret = null;
    public static Pose lastPose = new Pose(0,0,0);
    public static TurretConstants.SIDES lastSide = TurretConstants.SIDES.BLUE;
    private static final Pose redResetPose = new Pose(0, 0, Math.toRadians(0));
    public static Pose getResetPose() {
        if (lastSide == TurretConstants.SIDES.BLUE) {
            return redResetPose.mirror();
        }
        return redResetPose;
    }
    public static void applyReset(Follower follower) {
        Pose resetPose = getResetPose();
        lastPose = resetPose;
        follower.setPose(resetPose);
    }
}
