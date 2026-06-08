package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class TurretConstants {

    public static final String HMTurret = "turret";

    public static final String HMShooter1 = "shooter1";
    public static final String HMShooter2 = "shooter2";

    // GoBilda motor: 312 ticks per revolution
    public static final double ENCODER_CPR = 312.0;

    // No external gear ratio
    public static final double TURRET_GEAR_RATIO = 1.0;

    // CALIBRATION: Set this by rotating turret to point at goal, read encoder ticks, then:
    // turretEncoderOffset = -1 * (encoderTicks / 312.0) * 2.0 * Math.PI
    public static double turretEncoderOffset = 0.0;

    public static double turretP = 0.0;
    public static double turretI = 0.0;
    public static double turretD = 0.0;
    public static double turretF = 0.0;

    public static final double TURRET_HARD_LIMIT_RADIANS = Math.toRadians(130.0);
    public static final double TURRET_SOFT_LIMIT_RADIANS = Math.toRadians(128.0);

    public static final Pose blueGoalPose = new Pose(0, 144, 0);
    public static final Pose redGoalPose = blueGoalPose.mirror();

    public static double redOffset  = -Math.toRadians(0);
    public static double blueOffset =  Math.toRadians(0);

    public static SIDES selectedSide = SIDES.BLUE;

    // Shooting distance range in inches
    public static double minShootingDistance = 24.0;  // 2 feet
    public static double maxShootingDistance = 240.0; // 20 feet

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
