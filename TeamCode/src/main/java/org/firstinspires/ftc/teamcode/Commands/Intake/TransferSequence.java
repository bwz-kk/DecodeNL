package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Gate.CloseGate;
import org.firstinspires.ftc.teamcode.Commands.Gate.OpenGate;
import org.firstinspires.ftc.teamcode.Subsystem.Gate.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret;

public class TransferSequence extends SequentialCommandGroup {


    public TransferSequence(Intake intake, Gate gate, Turret turret){



        addCommands(
                new IntakeOff(intake),
                new CloseGate(gate),
                new WaitCommand(500),
                new IntakeOn(intake),

                new WaitCommand(600),
                new OpenGate(gate),
                new IntakeOff(intake)

        );
        addRequirements(intake, gate);
    }

}