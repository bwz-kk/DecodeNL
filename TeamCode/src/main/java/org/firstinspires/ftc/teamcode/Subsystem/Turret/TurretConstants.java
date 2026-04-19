package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import com.pedropathing.geometry.Pose;

public class TurretConstants {
    public static final String HMTurret = "turret";
    public static final Pose blueGoalPose = new Pose(0,144,0);
    public static final Pose redGoalPose = blueGoalPose.mirror();
    public static double redOffset = -Math.toRadians(7);
    public static double blueOffset = Math.toRadians(2);

}
