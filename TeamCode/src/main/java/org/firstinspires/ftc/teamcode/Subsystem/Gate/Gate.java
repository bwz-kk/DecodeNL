package org.firstinspires.ftc.teamcode.Subsystem.Gate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Gate extends SubsystemBase {
    Servo gate;
    public static final double position = 0;

    private final Telemetry telemetry;

    public Gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, GateConstants.HMGate);
        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    public double getPosition() {
        return gate.getPosition();
    }

    public void Open(){
        gate.setPosition(GateConstants.GateOpen);
    }
    public void Close(){
        gate.setPosition(GateConstants.GateClosed);
    }

    @Override
    public void periodic(){
        telemetry.addData("[Gate] Position", getPosition());
        telemetry.update();
    }
}
