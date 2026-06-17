package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOff;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOn;
import org.firstinspires.ftc.teamcode.Commands.Intake.TransferSequence;
import org.firstinspires.ftc.teamcode.Subsystem.Gate.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Indicator.Indicator;
import org.firstinspires.ftc.teamcode.Subsystem.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Side Auto")
public class FarSideAuto extends CommandOpMode {
    Turret turret;
    Intake intake;
    Gate gate;
    Indicator indicator;

    Follower follower;

    Path startToShoot;

    PathChain thirdRowChain;
    Path shootToHP;
    Path hpToShoot;
    Path shootToHPGate;
    Path hpGateToShoot;
    public Pose shootingPose;
    public Path shootToLeave;


    @Override
    public void initialize() {
        indicator = new Indicator(hardwareMap);
        turret = new Turret(hardwareMap, indicator);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        gate.Open();
        TurretConstants.SIDES side = null;

        telemetry.addLine("Press X for blue side, B for red side: ");
        telemetry.update();
        while (side == null) {
            if (gamepad1.x) {
                side = TurretConstants.SIDES.BLUE;
            } else if (gamepad1.b) {
                side = TurretConstants.SIDES.RED;
            }
        }
        telemetry.update();

        PosePersistency.selectedSide = side;
        turret.setSide(side);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        super.reset();

        follower = Constants.createFollower(hardwareMap);

        Pose startPose      = new Pose(61 ,8, Math.toRadians(180));
        Pose shootingPose   = new Pose(57.5, 22, Math.toRadians(180));

        Pose startThirdRow  = new Pose(50, 38, Math.toRadians(190));
        Pose endThirdRow    = new Pose(17, 38, Math.toRadians(190));

        Pose hpIntake       = new Pose(10, 7.5, Math.toRadians(190));
        Pose hpGateIntake   =  new Pose (7, 45, Math.toRadians(90));
        Pose hpControlToGateIntake = new Pose(5, 2, 0);

        Pose Leave = new Pose(18, 12, Math.toRadians(180));


        if (side == TurretConstants.SIDES.RED) {
            startPose           = startPose.mirror();
            shootingPose        = shootingPose.mirror();
            startThirdRow       = startThirdRow.mirror();
            endThirdRow         = endThirdRow.mirror();
            hpIntake            = hpIntake.mirror();
            hpGateIntake        = hpGateIntake.mirror();
            hpControlToGateIntake = hpControlToGateIntake.mirror();
            Leave               = Leave.mirror();
        }

        follower.setPose(startPose);

        startToShoot = new Path(new BezierLine(startPose, shootingPose));
        startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading());

        thirdRowChain = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, startThirdRow))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), startThirdRow.getHeading())
                .addPath(new BezierLine(startThirdRow, endThirdRow))
                .setLinearHeadingInterpolation(startThirdRow.getHeading(), endThirdRow.getHeading())
                .addPath(new BezierLine(endThirdRow, shootingPose))
                .setLinearHeadingInterpolation(endThirdRow.getHeading(), shootingPose.getHeading())
                .build();


        shootToHP = new Path(new BezierLine(shootingPose, hpIntake));
        shootToHP.setLinearHeadingInterpolation(shootingPose.getHeading(), hpIntake.getHeading());

        hpToShoot = new Path(new BezierLine(hpIntake, shootingPose));
        hpToShoot.setLinearHeadingInterpolation(hpIntake.getHeading(), shootingPose.getHeading());

        shootToHPGate = new Path(new BezierCurve(shootingPose, hpControlToGateIntake, hpGateIntake));
        shootToHPGate.setLinearHeadingInterpolation(shootingPose.getHeading(), hpGateIntake.getHeading());

        hpGateToShoot = new Path(new BezierCurve(hpGateIntake, hpControlToGateIntake, shootingPose));
        hpGateToShoot.setLinearHeadingInterpolation(hpGateIntake.getHeading(), shootingPose.getHeading());

        shootToLeave = new Path(new BezierLine(shootingPose, Leave));
        shootToLeave.setLinearHeadingInterpolation(shootingPose.getHeading(), Leave.getHeading());


        schedule(
                new RunCommand(() -> follower.update()),
                new RunCommand(() -> PosePersistency.savedPose = follower.getPose()),
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, startToShoot),
                        new TransferSequence(intake, gate, turret),

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, thirdRowChain),
                        new IntakeOff(intake),
                        new WaitCommand(300),
                        new TransferSequence(intake, gate, turret),

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, shootToHP),
                        new WaitCommand(500),
                        new IntakeOff(intake),
                        new FollowPathCommand(follower, hpToShoot),
                        new WaitCommand(300),
                        new TransferSequence(intake, gate, turret),

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, shootToHPGate),
                        new WaitCommand(1000),
                        new IntakeOff(intake),
                        new FollowPathCommand(follower, hpGateToShoot),
                        new WaitCommand(300),
                        new TransferSequence(intake, gate, turret),

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, shootToHP),
                        new WaitCommand(500),
                        new IntakeOff(intake),
                        new FollowPathCommand(follower, hpToShoot),
                        new WaitCommand(300),
                        new TransferSequence(intake, gate, turret),
                        new IntakeOn(intake),
                        new FollowPathCommand(follower, shootToHP),
                        new WaitCommand(500),
                        new IntakeOff(intake),
                        new FollowPathCommand(follower, hpToShoot),
                        new WaitCommand(300),
                        new TransferSequence(intake, gate, turret),

                        new FollowPathCommand(follower, shootToLeave)





                )
        );

        PosePersistency.turret = turret;
        register(turret, gate, intake, indicator);
        telemetry.addLine("Ready to run");
        telemetry.update();
    }

    @Override
    public void run() {
        PosePersistency.savedPose = follower.getPose();
        turret.updateBotPose(follower.getPose());
        telemetry.addData("Distance", turret.getDistance());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
        super.run();
    }
}