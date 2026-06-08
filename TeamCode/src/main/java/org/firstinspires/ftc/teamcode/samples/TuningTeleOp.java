package org.firstinspires.ftc.teamcode.samples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Subsystem.Gate.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Tuning TeleOp", group = "Tuning")
public class TuningTeleOp extends CommandOpMode {
    private Follower follower;
    private Turret turret;
    private Gate gate;
    private TelemetryData telemetryData;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(112, 135, Math.toRadians(-90)));
        turret = new Turret(hardwareMap);
        gate = new Gate(hardwareMap);

        TurretConstants.selectedSide = TurretConstants.SIDES.RED;
        turret.setSide(TurretConstants.SIDES.RED);

        telemetryData = new TelemetryData(telemetry);
        super.reset();
        follower.startTeleopDrive();
    }

    @Override
    public void run() {
        super.run();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();
        turret.updateBotPose(follower.getPose());

        if (gamepad1.a) {
            gate.Open();
        } else if (gamepad1.b) {
            gate.Close();
        }

        if (gamepad1.x) {
            if (Turret.tuningVelocity > 0) {
                turret.setShooterVelocity(Turret.tuningVelocity);
            } else {
                turret.setShooterVelocity(0);
            }
        } else if (gamepad1.y) {
            turret.setShooterVelocity(0);
        }

        if (gamepad1.dpad_up) {
            turret.recordCalibrationPoint();
        }
        if (gamepad1.dpad_down) {
            turret.clearCalibrationPoints();
        }

        turret.periodic();
        gate.periodic();

        telemetryData.addData("Drive X",           follower.getPose().getX());
        telemetryData.addData("Drive Y",           follower.getPose().getY());
        telemetryData.addData("Drive Heading",     follower.getPose().getHeading());
        telemetryData.addData("Distance to Goal",  turret.getDistance());
        telemetryData.addData("Calibration Points", turret.getCalibrationPointCount());
        telemetryData.addData("Last Recorded",     turret.getLastCalibrationSummary());

        if (gamepad1.dpad_left) {
            telemetry.addData("--- Calibration Code ---", "");
            for (String line : turret.getCalibrationCode().split("\n")) {
                telemetry.addData("  ", line);
            }
        }

        telemetryData.update();
    }
}