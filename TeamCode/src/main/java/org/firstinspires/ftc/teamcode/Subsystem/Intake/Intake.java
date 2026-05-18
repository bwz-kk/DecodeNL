package org.firstinspires.ftc.teamcode.Subsystem.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake extends SubsystemBase {

    DcMotor Intake1;
    double currentPower = 0;

    public Intake(HardwareMap hardwareMap) {
        Intake1 = hardwareMap.get(DcMotor.class, IntakeConstants.HMIntake1);
    }

    public void TurnOnIntake() {
        currentPower = IntakeConstants.TurnOn;
    }

    public void TurnIntakeOff() {
        currentPower = IntakeConstants.TurnOff;
    }

    @Override
    public void periodic() {
        Intake1.setPower(currentPower);
    }
}
