package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import com.pedropathing.geometry.Pose;

public class TurretConstants {

    public static final String HMTurret = "turret";

    public static final String HMTurretEncoder = "turretEncoder";

    public static final String HMShooter1 = "shooter1";

    public static final String HMShooter2 = "shooter2";

    /**
     * Maximum analog voltage output of the REV Through Bore Encoder V1.
     * The encoder outputs 0 V at 0° and MAX_ENCODER_VOLTAGE at 360°.
     * REV spec: 3.3 V full-scale output.
     */
    public static final double MAX_ENCODER_VOLTAGE = 3.3;

    public static final double TURRET_GEAR_RATIO = 1.0;

    public static double turretEncoderOffset = 0.0;

    public static final Pose blueGoalPose = new Pose(0, 144, 0);

    public static final Pose redGoalPose = blueGoalPose.mirror();

    public static double redOffset = -Math.toRadians(7);
    public static double blueOffset = Math.toRadians(2);
    public static final double TURRET_HARD_LIMIT_RADIANS = Math.toRadians(130.0);
    public static final double TURRET_SOFT_LIMIT_RADIANS = Math.toRadians(128.0);

    public static SIDES selectedSide = SIDES.BLUE;

    public enum SIDES {
        BLUE,
        RED
    }
    public static Pose getGoalPose(SIDES side) {
        switch (side) {
            case RED:
                return redGoalPose;
            case BLUE:
            default:
                return blueGoalPose;
        }
    }
}