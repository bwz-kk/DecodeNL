package org.firstinspires.ftc.teamcode.Competition;

import static org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants.selectedSide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOff;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOn;
import org.firstinspires.ftc.teamcode.Commands.Intake.TransferSequence;
import org.firstinspires.ftc.teamcode.Commands.Vision.UpdatePoseCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Gate.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Indicator.Indicator;
import org.firstinspires.ftc.teamcode.Subsystem.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.Subsystem.Vision.VisionOdometry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Optional;

@TeleOp(name = "DecodeNL TeleOp", group = "Match")
public class DecodeNLTeleOp extends CommandOpMode {
    private Turret turret;
    private Intake intake;
    private Gate gate;
    private Follower follower;
    private TelemetryData telemetryData;
    private Indicator indicator;


    @Override
    public void initialize() {
        GamepadEx gamepadEx =  new GamepadEx(gamepad1);

        follower = Constants.createFollower(hardwareMap);
        indicator = new Indicator(hardwareMap);

        if (PosePersistency.turret == null) {
            turret = new Turret(hardwareMap, indicator);
        } else {
            turret = PosePersistency.turret;
            turret.reinitMotors();
        }

        if (PosePersistency.savedPose != null) {
            follower.setStartingPose(PosePersistency.savedPose);
        }

        TurretConstants.selectedSide = PosePersistency.selectedSide;

        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);



        turret.setSide(PosePersistency.selectedSide);
        follower.setPose(new Pose(112, 135, Math.toRadians(-90)));
        //follower.setPose(PosePersistency.savedPose);

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(new IntakeOn(intake));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(new IntakeOff(intake));
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(new TransferSequence(intake, gate, turret));
        register(turret, intake, gate, indicator);
        follower.startTeleOpDrive();
    }
    @Override
    public void run(){
        super.run();
        follower.setTeleOpDrive(gamepad1.left_stick_y * (PosePersistency.selectedSide == TurretConstants.SIDES.RED ? -1 : 1), gamepad1.left_stick_x * (PosePersistency.selectedSide == TurretConstants.SIDES.RED ? -1 : 1), -gamepad1.right_stick_x, false);
        follower.update();

        turret.updateBotPose(follower.getPose());
    }


}