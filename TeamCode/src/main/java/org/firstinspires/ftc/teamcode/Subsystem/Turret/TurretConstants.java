package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import com.pedropathing.geometry.Pose;

public class TurretConstants {
    public static final String TauraServo1 = "TauraServo1";
    public static final String TauraServo2 = "TauraServo2";
    public static final String TauraServo3 = "TauraServo3";
    public static final String HMThroughBore = "throughBoreEncoder";
    public static final String HMShooter1 = "shooter1";
    public static final String HMShooter2 = "shooter2";
    public static final String HMHood = "hood";
    public static final Pose blueGoalPose = new Pose(0,144,0);
    public static final Pose redGoalPose = blueGoalPose.mirror();
    public static double redOffset = -Math.toRadians(7);
    public static double blueOffset = Math.toRadians(2);
    public static final double hoodMinPosition = 0;
    public static final double hoodMaxPosition = 0.7;
    public static double THROUGH_BORE_TICKS_PER_REV = 8192.0;
    public static long   turretZeroOffsetTicks       = 0;

    public static double turretMinAngle    = -Math.PI;  // tune to physical limit
    public static double turretMaxAngle    =  Math.PI;  // tune to physical limit
    public static double turretMinPosition = 0.0;
    public static double turretMaxPosition = 1.0;


    public enum SIDES {
        BLUE,
        RED
    }


}
