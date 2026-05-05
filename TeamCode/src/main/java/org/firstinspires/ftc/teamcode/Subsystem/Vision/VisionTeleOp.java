package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.util.TelemetryData;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class VisionTeleOp extends CommandOpMode {

    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    private VisionOdometry visionOdometry;

    @Override
    public void initialize() {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        follower = Constants.createFollower(hardwareMap);

        visionOdometry = new VisionOdometry();
        visionOdometry.init(hardwareMap, telemetry);

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenActive(() -> visionOdometry.resetPoseFromTag(follower));

        telemetry = FtcDashboard.getInstance().getTelemetry();
        follower.startTeleOpDrive();
    }


    @Override
    public void run() {
        super.run();

        visionOdometry.update();

        follower.setTeleOpDrive(gamepad1.left_stick_y , gamepad1.left_stick_x , -gamepad1.right_stick_x, false);

        follower.update();

        telemetryData.addData("Pose robô X", follower.getPose().getX());
        telemetryData.addData("Pose robô Y", follower.getPose().getY());
        telemetryData.addData("Tag válida: ", visionOdometry.hasValidTag());
        telemetryData.update();
    }
}
