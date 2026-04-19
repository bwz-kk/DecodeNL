package org.firstinspires.ftc.teamcode.Commands.Gate;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.Gate.Gate;

public class OpenGate extends CommandBase {

    private final Gate gate;

    public OpenGate(Gate gate) {
        this.gate = gate;
    }
    @Override
    public void initialize() {
        gate.Open();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

