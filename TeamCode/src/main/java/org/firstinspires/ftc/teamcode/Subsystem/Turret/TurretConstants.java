package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import com.pedropathing.geometry.Pose;

public class TurretConstants {

    public static final String HMTurret = "turret";

    public static final String HMShooter1 = "shooter1";
    public static final String HMShooter2 = "shooter2";

    // ── Encoder Constants ──────────────────────────────────────────────────────
    //
    // The REV Through Bore Encoder V1 is wired as a quadrature encoder on the
    // turret motor port.  In quadrature (4×) mode the encoder produces:
    //
    //     CPR = 8192  ticks per output-shaft revolution
    //
    // (The bare encoder has 2048 cycles/rev; quadrature decoding multiplies by
    //  four.)
    //
    // GEAR RATIO
    //   TURRET_GEAR_RATIO = 1.0  indicates the encoder shaft rotates at the
    //   same speed as the turret output (1:1).  If there is a gearbox between
    //   the encoder and the turret, update this value.
    //
    //   Actual turret angle = encoderTicks / ENCODER_CPR × 2π / TURRET_GEAR_RATIO

    /** REV Through Bore Encoder V1 counts per revolution (quadrature 4×). */
    public static final double ENCODER_CPR = 8192.0;

    /**
     * Gear ratio from encoder shaft to turret output.
     * <p>
     * ASSUMPTION: 1.0 — the encoder is mounted directly on the turret output
     * shaft (no additional gearbox).  Verify on the physical robot.
     */
    public static final double TURRET_GEAR_RATIO = 1.0;

    /**
     * Angular offset (radians) added to the raw encoder reading so that the
     * turret heading corresponds to the physical zero position.
     * <p>
     * Tune by pointing the turret at a known reference, reading the raw angle,
     * and setting offset = −rawAngle.
     */
    public static double turretEncoderOffset = 0.0;

    // ── Mechanical Limits ──────────────────────────────────────────────────────
    //
    // The turret has ±130° of mechanical travel (hard stops).  Soft limits at
    // ±128° give a 2° safety margin before the hard stop.

    public static final double TURRET_HARD_LIMIT_RADIANS = Math.toRadians(130.0);
    public static final double TURRET_SOFT_LIMIT_RADIANS = Math.toRadians(128.0);

    // ── Goal Poses ─────────────────────────────────────────────────────────────

    public static final Pose blueGoalPose = new Pose(0, 144, 0);
    public static final Pose redGoalPose = blueGoalPose.mirror();

    public static double redOffset  = -Math.toRadians(7);
    public static double blueOffset =  Math.toRadians(2);

    // ── Side Enum ──────────────────────────────────────────────────────────────

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