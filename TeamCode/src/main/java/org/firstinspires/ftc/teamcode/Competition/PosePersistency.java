package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants;

public class PosePersistency {
    public static Turret turret = null;

    public static Pose savedPose = null;

    public static TurretConstants.SIDES selectedSide = TurretConstants.SIDES.RED;

    public static void reset() {
        turret = null;
        savedPose = null;
        selectedSide = TurretConstants.SIDES.BLUE;
    }
}

