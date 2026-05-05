package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import com.pedropathing.geometry.Pose;

public class TurretConstants {
    public static final String HMTurret = "turret";
    public static final String HMShooter1 = "shooter1";
    public static final String HMShooter2 = "shooter2";
    public static final String HMHood = "hood";
    public static final Pose blueGoalPose = new Pose(0,144,0);
    public static final Pose redGoalPose = blueGoalPose.mirror();
    public static double redOffset = -Math.toRadians(7);
    public static double blueOffset = Math.toRadians(2);
    public static final double hoodMinPosition = 0;
    public static final double hoodMaxPosition = 0.7;
    public enum SIDES {
        BLUE,
        RED
    }


}
