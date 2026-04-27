package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class VisionTeleOp extends CommandOpMode {

    private Follower follower;

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

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

        visionOdometry.update();

        follower.setTeleOpDrive(gamepad1.left_stick_y , gamepad1.left_stick_x , -gamepad1.right_stick_x, false);

        follower.update();

        telemetry.addData("Pose robô X", String.format("%.1f\"", follower.getPose().getX()));
        telemetry.addData("Pose robô Y", String.format("%.1f\"", follower.getPose().getY()));
        telemetry.addData("Tag válida: ", visionOdometry.hasValidTag());
        telemetry.update();
    }
}
