package org.firstinspires.ftc.teamcode.Hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

public class TauraServo {

    private static final double A = -0.06637714;
    private static final double B =  0.34132813;

    private static final double MAX_ANGLE_DEGREES_ANGULAR = 315;
    private static final double MAX_ANGLE_DEGREES_CONTINOUS = 330;

    private double lastRawAngleInDegrees = 0.0;
    private double deltaPositionInDegrees = 0.0;
    private AnalogInput analogFeedbackSensor;

    private Servo baseServo = null;

    public TauraServo(@NonNull Servo base) {
        baseServo = base;
    }

    public void setPosition(double position) { baseServo.setPosition(position); }

    public double getPosition() { return baseServo.getPosition(); }

    public void scaleRange(double min, double max) { baseServo.scaleRange(min, max); }

    public void setDirection(Servo.Direction direction) { baseServo.setDirection(direction); }

    public void setAnalogFeedbackSensor(AnalogInput analogFeedbackSensor) {
        this.analogFeedbackSensor = analogFeedbackSensor;
    }

    public double getIncrementalPositionInDegrees() {
        double actualAngleInDegrees = getRawPositionInDegrees();

        double delta = actualAngleInDegrees - lastRawAngleInDegrees;
        double halfRange = MAX_ANGLE_DEGREES_CONTINOUS / 2.0;

        if (delta >  halfRange) delta -= MAX_ANGLE_DEGREES_CONTINOUS;
        else if (delta < -halfRange) delta += MAX_ANGLE_DEGREES_CONTINOUS;

        deltaPositionInDegrees += delta;
        lastRawAngleInDegrees = actualAngleInDegrees;

        return deltaPositionInDegrees;
    }

    public double getRawPositionInDegrees() {
        return getUniversalPosition() * MAX_ANGLE_DEGREES_ANGULAR;
    }

    public double getUniversalPosition() {
        return A + B * getVoltageFromAnalogSensor();
    }

    private double getVoltageFromAnalogSensor() {
        try {
            return analogFeedbackSensor.getVoltage();
        } catch(NullPointerException e) {
            return Double.NaN;
        }
    }

}
