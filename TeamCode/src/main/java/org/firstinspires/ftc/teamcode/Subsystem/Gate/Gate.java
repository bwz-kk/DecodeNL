package org.firstinspires.ftc.teamcode.Subsystem.Gate;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Gate extends SubsystemBase {
    Servo gate;
    public static final double position = 0;

    public Gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, GateConstants.HMGate);
    }

    public void Open(){
        gate.setPosition(GateConstants.GateOpen);
    }
    public void Close(){
        gate.setPosition(GateConstants.GateClosed);
    }

    @Override
    public void periodic(){
    }
}
