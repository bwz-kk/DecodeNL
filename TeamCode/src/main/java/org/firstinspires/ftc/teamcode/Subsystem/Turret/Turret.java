package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Field.SIDES;

public class Turret extends SubsystemBase {
    public static int tuningVelocity = 0;
    DcMotorEx turret;
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    InterpLUT velocityInterpolation = new InterpLUT();
    double minDistance = 0;
    double maxDistance = 0;

    Pose lastPose = new Pose(0, 0, 0);
    Pose botPose = new Pose(0, 0, 0);
    Pose poseToAim = new Pose(0, 0, 0);
    Pose virtualBotPose = new Pose(0, 0, 0);

    private Vector movementVector = new Vector(0, 0);
    double virtualBotMultiplier = 2;

    double distance = 0;
    double targetAngleFC = 0;

    public PIDController turretController = new PIDController(0, 0, 0);
    SIDES sides = SIDES.BLUE;
    public Turret(HardwareMap hardwareMap) {

    }


}
