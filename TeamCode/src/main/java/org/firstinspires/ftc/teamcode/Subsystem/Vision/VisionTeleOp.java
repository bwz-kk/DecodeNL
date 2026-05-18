package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.util.TelemetryData;


import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOff;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOn;
import org.firstinspires.ftc.teamcode.Field.DashboardDrawing;
import org.firstinspires.ftc.teamcode.Subsystem.Intake.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class VisionTeleOp extends CommandOpMode {
    private Follower follower;
    private Intake intake;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    private VisionOdometry visionOdometry;
    private boolean justResetPose = false;

    @Override
    public void initialize() {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);


        visionOdometry = new VisionOdometry();
        visionOdometry.init(hardwareMap, telemetry);
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(new IntakeOn(intake));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(new IntakeOff(intake));
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenActive(() -> {
            if (visionOdometry.resetPoseFromTag(follower)) {
                justResetPose = true;
                telemetry.addLine("pose reset from tag");
            } else {
                telemetry.addLine("no valid tag");
            }
        });

        telemetry = FtcDashboard.getInstance().getTelemetry();
        follower.startTeleOpDrive();

    }


    @Override
    public void run() {

        super.run();

        visionOdometry.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.getPose();
        follower.update();

        TelemetryPacket packet = new TelemetryPacket();
        DashboardDrawing.drawDebug(packet.fieldOverlay(), follower);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryData.addData("Pose robô X", follower.getPose().getX());
        telemetryData.addData("Pose robô Y", follower.getPose().getY());
        telemetryData.addData("Tag válida: ", visionOdometry.hasValidTag());
        if (visionOdometry.hasValidTag()) {
            Pose visionPose = visionOdometry.getCurrentVisionPose();
            telemetryData.addData("Vision X", visionPose.getX());
            telemetryData.addData("Vision Y", visionPose.getY());
            telemetryData.addData("Vision Heading", visionPose.getHeading());
        }
        if (justResetPose) {
            telemetryData.addData("Pose reset status", "just reset");
            justResetPose = false;
        }
        telemetryData.update();
    }
}
