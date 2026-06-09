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
    public static double tuningPosition = 0.0;

    private double commandedPosition = -1.0;
    private final Telemetry telemetry;

    public Gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, GateConstants.HMGate);
        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    public double getPosition() {
        return gate.getPosition();
    }

    public void setPosition(double position) {
        commandedPosition = position;
        gate.setPosition(position);
    }

    public void Open(){
        setPosition(GateConstants.GateOpen);
    }
    public void Close(){
        setPosition(GateConstants.GateClosed);
    }

    @Override
    public void periodic(){
        if (tuningPosition > 0.0) {
            gate.setPosition(tuningPosition);
        } else if (commandedPosition >= 0.0) {
            gate.setPosition(commandedPosition);
        }

        telemetry.addData("[Gate] Position", getPosition());
        telemetry.addData("[Gate] Commanded", commandedPosition);
        telemetry.addData("[Gate] Tuning", tuningPosition);
        telemetry.update();
    }
}
