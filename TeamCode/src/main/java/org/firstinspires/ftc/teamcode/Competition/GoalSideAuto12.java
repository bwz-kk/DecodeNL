package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Goal Side Auto 12")
public class GoalSideAuto12 extends CommandOpMode {
    Turret turret;
    Intake intake;
    Gate gate;
    Indicator indicator;

    Follower follower;

    Path startToShoot;

    PathChain firstRowChain;
    PathChain secondRowChain;
    PathChain thirdRowChain;

    Path gateIntakeToShoot;
    Path shootToGateIntake;
    Path shootToGateIntake2;
    PathChain gateIntakeChain;
    public Pose shootingPose;
    public Pose shootingPose2;
    public Path shootToLeave;
    PathChain shootToGateChain;

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

        Pose startPose = new Pose(31.7, 144-8.5, Math.toRadians(-90));

        shootingPose = new Pose(57, 81, Math.toRadians(-160));
        shootingPose2 = new Pose(55, 100, Math.toRadians(-150));
        Pose startFirstRow = new Pose(43, 84, Math.toRadians(190));
        Pose endFirstRow   = new Pose(23, 84, Math.toRadians(190));

        Pose startSecondRow = new Pose(43, 63, Math.toRadians(180));
        Pose endSecondRow   = new Pose(19, 63, Math.toRadians(180));

        Pose startThirdRow = new Pose(43, 40, Math.toRadians(180));
        Pose endThirdRow   = new Pose(19, 40, Math.toRadians(180));

        Pose gateIntake = new Pose(9, 62, Math.toRadians(160));
        Pose gateControl = new Pose(15,57, 0);
        Pose gateIntake2 = new Pose(15,53, Math.toRadians(90));

        Pose openGate = new Pose(12, 70, Math.toRadians(-0));

        Pose Leave = new Pose(45, 81, Math.toRadians(-170));

        if (side == TurretConstants.SIDES.RED) {
            startPose = startPose.mirror();
            shootingPose2 = shootingPose2.mirror();
            shootingPose = shootingPose.mirror();
            startFirstRow = startFirstRow.mirror();
            endFirstRow = endFirstRow.mirror();
            startSecondRow = startSecondRow.mirror();
            endSecondRow = endSecondRow.mirror();
            startThirdRow = startThirdRow.mirror();
            endThirdRow = endThirdRow.mirror();
            gateIntake = gateIntake.mirror();
            gateIntake2 = gateIntake2.mirror();
            openGate = openGate.mirror();
            Leave = Leave.mirror();
        }

        follower.setPose(startPose);

        startToShoot = new Path(new BezierLine(startPose, shootingPose));
        //startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading());

        // First row: shoot -> startRow -> endRow -> shoot, all in one chain
        Pose finalStartFirstRow = startFirstRow;
        Pose finalEndFirstRow = endFirstRow;
        firstRowChain = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, finalStartFirstRow))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), finalStartFirstRow.getHeading())
                .addPath(new BezierLine(finalStartFirstRow, finalEndFirstRow))
                .setLinearHeadingInterpolation(finalStartFirstRow.getHeading(), finalEndFirstRow.getHeading())
                .addPath(new BezierLine(finalEndFirstRow, shootingPose))

                .setLinearHeadingInterpolation(finalEndFirstRow.getHeading(), shootingPose.getHeading())
                .build();

        // Second row: shoot -> startRow -> endRow -> shoot, all in one chain
        Pose finalStartSecondRow = startSecondRow;
        Pose finalEndSecondRow = endSecondRow;
        secondRowChain = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, finalStartSecondRow))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), finalStartSecondRow.getHeading())
                .addPath(new BezierLine(finalStartSecondRow, finalEndSecondRow))
                .setLinearHeadingInterpolation(finalStartSecondRow.getHeading(), finalEndSecondRow.getHeading())
                .addPath(new BezierLine(finalEndSecondRow, shootingPose))
                .setReversed()
                //.setLinearHeadingInterpolation(finalEndSecondRow.getHeading(), shootingPose.getHeading())
                .build();

        // Third row: shoot -> startRow -> endRow -> shoot, all in one chain
        Pose finalStartThirdRow = startThirdRow;
        Pose finalEndThirdRow = endThirdRow;
        thirdRowChain = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, finalStartThirdRow))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), finalStartThirdRow.getHeading())
                .addPath(new BezierLine(finalStartThirdRow, finalEndThirdRow))
                .setLinearHeadingInterpolation(finalStartThirdRow.getHeading(), finalEndThirdRow.getHeading())
                .addPath(new BezierLine(finalEndThirdRow, shootingPose))
                .setReversed()
                //.setLinearHeadingInterpolation(finalEndThirdRow.getHeading(), shootingPose.getHeading())
                .build();

        gateIntakeToShoot = new Path(new BezierLine(gateIntake2, shootingPose));
        gateIntakeToShoot.reverseHeadingInterpolation();
        //gateIntakeToShoot.setLinearHeadingInterpolation(gateIntake.getHeading(), shootingPose.getHeading());

        shootToGateIntake = new Path(new BezierLine(shootingPose, gateIntake));
        shootToGateIntake.setLinearHeadingInterpolation(shootingPose.getHeading(), gateIntake.getHeading());

        shootToLeave = new Path(new BezierLine(shootingPose, Leave));
        shootToLeave.setLinearHeadingInterpolation(shootingPose.getHeading(), Leave.getHeading());

        shootToGateChain = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, openGate))
                .setLinearHeadingInterpolation(openGate.getHeading(), openGate.getHeading())
                .addPath(new BezierLine(openGate, shootingPose))
                .setLinearHeadingInterpolation(openGate.getHeading(), openGate.getHeading())
                .build();


        //startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading());

        gateIntakeChain = follower.pathBuilder().addPath(shootToGateIntake).build();
        int gateDelay = 250;
        schedule(
                new RunCommand(() -> follower.update()),
                new RunCommand(() -> PosePersistency.savedPose= follower.getPose()),
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, startToShoot),
                        new TransferSequence(intake, gate, turret),

                        // First row: robot drives shoot->row->sweep->shoot without stopping

                        // Second row: same pattern
                        new IntakeOn(intake),
                        new FollowPathCommand(follower, secondRowChain),
                        new IntakeOff(intake),
                        //new FollowPathCommand(follower, shootToGateChain),
                        new TransferSequence(intake, gate, turret),

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, firstRowChain),
                        new IntakeOff(intake),
                        new TransferSequence(intake, gate, turret),

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, thirdRowChain),
                        new IntakeOff(intake),
                        new TransferSequence(intake, gate, turret),


                        // Gate intake sequences (unchanged)



                        /*new IntakeOn(intake),
                        new FollowPathCommand(follower, shootToGateIntake),
                        new IntakeTransfer(intake),
                        new WaitCommand(gateDelay),
                        new FollowPathCommand(follower, gateIntakeToShoot),
                        new TransferSequence(intake, gate, turret),*/

                        new FollowPathCommand(follower, shootToLeave)

                )
        );

        PosePersistency.turret = turret;
        register(turret, gate, intake);
        telemetry.addLine("Ready to run");
        telemetry.update();
    }

    @Override
    public void run() {
        turret.updateBotPose(follower.getPose());
        PosePersistency.savedPose = follower.getPose();
        telemetry.addData("Distance", turret.getDistance());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("OFFSET: ", TurretConstants.blueOffset);
        telemetry.update();
        super.run();
    }
}

