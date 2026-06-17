package org.firstinspires.ftc.teamcode.samples;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystem.Indicator.Indicator;
import org.firstinspires.ftc.teamcode.Subsystem.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp
public class TurretTuningTeleOp extends LinearOpMode {

    public static double PID_P = 3;
    public static double PID_I = 0;
    public static double PID_D = 0;
    public static int targetVelocity = 0;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(112, 135, Math.toRadians(-90)));
        Indicator indicator = new Indicator(hardwareMap);
        Turret turret = new Turret(hardwareMap, indicator);
        Intake intake = new Intake(hardwareMap);
        telemetry.update();

        TurretConstants.selectedSide = TurretConstants.SIDES.RED;
        turret.setSide(TurretConstants.SIDES.RED);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            turret.updateBotPose(follower.getPose());
            intake.periodic();

            if (gamepad1.a){
                intake.TurnOnIntake();
            } else {
                intake.TurnIntakeOff();
            }

            turret.turretController.setPID(PID_P, PID_I, PID_D);
            turret.setShooterVelocity(targetVelocity);

            turret.periodic();

            double currentAngle = turret.getTurretAngle();

            telemetry.addLine("=== Turret Tuning ===");
            telemetry.addData("Actual Ângle (graus)", String.format("%.2f", Math.toDegrees(currentAngle)));
            telemetry.addData("Goal Distance (cm)", String.format("%.1f", turret.getDistance()));
            telemetry.addLine("--- PID (ativo) ---");
            telemetry.addData("  P", PID_P);
            telemetry.addData("  I", PID_I);
            telemetry.addData("  D", PID_D);
            telemetry.update();
        }
    }
}

