package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.Intake.Intake;

public class IntakeOff extends CommandBase {
    private final Intake intake;

    public IntakeOff(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.TurnIntakeOff();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}


