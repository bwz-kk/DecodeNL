package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Controller.PIDFController;
import org.firstinspires.ftc.teamcode.Hardware.TauraServo;

public class PIDServo {
    PIDFController controller;
    public TauraServo taura;
    public PIDServo(Servo servo, AnalogInput encoder, PIDFController pidf){
        taura = new TauraServo(servo);
        taura.setAnalogFeedbackSensor(encoder);
        controller = pidf;
    }

    DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

    public void setDirection(DcMotorSimple.Direction direction){
        this.direction = direction;
    }
    private double angularSP = 0;
    public void setAngularSP(double angularSP){

        this.angularSP =   angularSP;
    }
    public void update(){

        controller.setSetpoint(angularSP);
        double output = controller.calculate(taura.getIncrementalPositionInDegrees());
        output = 0.5+(Range.clip(output, -0.5, 0.5)*(direction== DcMotorSimple.Direction.REVERSE?-1:1));
        taura.setPosition(output);
    }


}
