package org.firstinspires.ftc.teamcode.Subsystem.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake extends SubsystemBase {
    DcMotor Intake1;
    DcMotor Intake2;


    public Intake(HardwareMap hardwareMap) {
        Intake1 = hardwareMap.get(DcMotor.class, IntakeConstants.HMIntake1);
        Intake2 = hardwareMap.get(DcMotor.class, IntakeConstants.HMIntake2);
    }

    public void TurnOnIntake(){
        Intake1.setPower(IntakeConstants.TurnOn);
        Intake2.setPower(IntakeConstants.TurnOn);
}

    public void TurnIntakeOff() {
        Intake1.setPower(IntakeConstants.TurnOff);
        Intake2.setPower(IntakeConstants.TurnOff);
    }

    @Override
public void periodic(){

    }
}
