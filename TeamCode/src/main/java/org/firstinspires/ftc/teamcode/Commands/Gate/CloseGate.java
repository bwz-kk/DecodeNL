package org.firstinspires.ftc.teamcode.Commands.Gate;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.Gate.Gate;

public class CloseGate extends CommandBase {

    private final Gate gate;

    public CloseGate(Gate gate) {
        this.gate = gate;
    }

    @Override
    public void initialize() {gate.Close();}

    @Override
    public boolean isFinished() {
        return true;
    }
}

