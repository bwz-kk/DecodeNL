package org.firstinspires.ftc.teamcode.samples;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Commands.Gate.CloseGate;
import org.firstinspires.ftc.teamcode.Commands.Gate.OpenGate;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOff;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOn;
import org.firstinspires.ftc.teamcode.Commands.Intake.TransferSequence;
import org.firstinspires.ftc.teamcode.Subsystem.Gate.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Indicator.Indicator;
import org.firstinspires.ftc.teamcode.Subsystem.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.VelocityCalibrationPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

/**
 * TuningTeleOp — InterpLUT Calibration Workflow
 *
 * Controls
 * ─────────────────────────────────────────────
 * Left stick XY / Right stick X  — drive (field-centric)
 * Left Bumper (hold)             — intake on / release = off
 * Right Bumper                   — transfer sequence
 * A / B                          — gate open / close
 * D-pad Up   (single press)      — record calibration point
 * D-pad Down (single press)      — clear all calibration points
 *
 * Calibration Workflow
 * ─────────────────────────────────────────────
 * 1. Run this OpMode.
 * 2. Drive robot to desired shooting position.
 * 3. Confirm the Live Preview values look correct (angle, distance).
 * 4. Press D-pad Up once — point is stored.
 * 5. Repeat for every distance you need.
 * 6. Read the CALIBRATION CODE block from FTC Dashboard.
 * 7. Paste the lut.add(...) lines into Turret.buildVelocityTable().
 */
@TeleOp(name = "Tuning TeleOp", group = "Tuning")
public class TuningTeleOp extends CommandOpMode {

    private Follower follower;
    private Turret  turret;
    Gate    gate;
    private Intake  intake;
    private GamepadEx gamepadEx;
    private TelemetryData telemetryData;
    Indicator indicator;


    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(112, 135, Math.toRadians(-90)));

        indicator = new Indicator(hardwareMap);
        turret = new Turret(hardwareMap, indicator);
        gate   = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);

        TurretConstants.selectedSide = TurretConstants.SIDES.RED;
        turret.setSide(TurretConstants.SIDES.RED);

        telemetryData = new TelemetryData(telemetry);
        gamepadEx = new GamepadEx(gamepad1);

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(new IntakeOn(intake));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(new IntakeOff(intake));
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(new TransferSequence(intake, gate, turret));
        register(intake, gate, turret, indicator);
        follower.startTeleOpDrive();

        super.reset();
    }

    @Override
    public void run() {
        super.run();

        follower.update();
        turret.updateBotPose(follower.getPose());

        turret.periodic();
        gate.periodic();
        intake.periodic();
        buildTelemetry();
    }

    private void buildTelemetry() {

        telemetryData.addData("Drive X",       follower.getPose().getX());
        telemetryData.addData("Drive Y",       follower.getPose().getY());
        telemetryData.addData("Drive Heading", Math.toDegrees(follower.getPose().getHeading()));

        telemetryData.addData("------- LIVE PREVIEW -------", "");
        telemetryData.addData("Turret Angle (deg)",   String.format("%.2f", Math.toDegrees(turret.getTurretAngle())));
        telemetryData.addData("Encoder Ticks",        turret.getTurretEncoderTicks());
        telemetryData.addData("Distance to Goal (in)", String.format("%.2f", turret.getDistance()));


        telemetryData.addData("----------------------------", "");

        telemetryData.addData("Manual Angle Mode", Turret.useManualAngle);
        telemetryData.addData("Manual Angle (deg)", String.format("%.1f", Math.toDegrees(Turret.manualAngle)));
        telemetryData.addData("PID kP", String.format("%.3f", Turret.kP));
        telemetryData.addData("PID kI", String.format("%.3f", Turret.kI));
        telemetryData.addData("PID kD", String.format("%.3f", Turret.kD));

        telemetryData.update();
    }

}