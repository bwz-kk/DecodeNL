package org.firstinspires.ftc.teamcode.Subsystem.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(new IntakeOn(intake));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenInactive(new IntakeOff(intake));

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenActive(() -> {
            Pose currentPose = follower.getPose();

            boolean mt2Success = visionOdometry
                    .getRobotPoseMT2(currentPose.getHeading())
                    .map(mt2Pose -> {
                        double distInches = Math.hypot(
                                mt2Pose.getX() - currentPose.getX(),
                                mt2Pose.getY() - currentPose.getY()
                        );
                        double maxDeltaInches =
                                OdometryConstants.MAX_DELTA_METERS
                                * OdometryConstants.METERS_TO_INCHES;

                        if (distInches < maxDeltaInches) {
                            double wOdo = OdometryConstants.ODOMETRY_WEIGHT;
                            double wLL = OdometryConstants.LIMELIGHT_WEIGHT;
                            double total = wOdo + wLL;

                            double fusedX = (currentPose.getX() * wOdo
                                    + mt2Pose.getX() * wLL) / total;
                            double fusedY = (currentPose.getY() * wOdo
                                    + mt2Pose.getY() * wLL) / total;

                            follower.setPose(new Pose(fusedX, fusedY, currentPose.getHeading()));
                            return true;
                        }
                        return false;
                    })
                    .orElse(false);

            if (!mt2Success) {
                visionOdometry.resetPoseFromTag(follower);
            }

            justResetPose = true;
            telemetry.addLine(mt2Success
                    ? "pose reset from tag (MT2)"
                    : "pose reset from tag (fiducial fallback)");
        });

        telemetry = FtcDashboard.getInstance().getTelemetry();
        follower.startTeleOpDrive();
    }

    @Override
    public void run() {
        super.run();

        visionOdometry.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();

        TelemetryPacket packet = new TelemetryPacket();
        DashboardDrawing.drawDebug(packet.fieldOverlay(), follower);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryData.addData("Pose robô X", follower.getPose().getX());
        telemetryData.addData("Pose robô Y", follower.getPose().getY());
        telemetryData.addData("Tag válida", visionOdometry.hasValidDetection());
        if (visionOdometry.hasValidDetection()) {
            Pose visionPose = visionOdometry.getCurrentVisionPose();
            if (visionPose != null) {
                telemetryData.addData("Vision X", visionPose.getX());
                telemetryData.addData("Vision Y", visionPose.getY());
                telemetryData.addData("Vision Heading", visionPose.getHeading());
            }
        }
        if (justResetPose) {
            telemetryData.addData("Pose reset status", "just reset");
            justResetPose = false;
        }
        telemetryData.update();
    }
}