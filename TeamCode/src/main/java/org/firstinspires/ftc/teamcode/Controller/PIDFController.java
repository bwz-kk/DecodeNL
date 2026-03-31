package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.util.Range;

public class PIDFController {
    // Ganhos
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    // Estado interno
    private double setpoint = 0.0;
    private double integralSum = 0.0;
    private double lastError = 0.0;

    // Limites
    private double integralLimit = 1.0;
    private double outputLimit = 1.0;

    // Tempo
    private long lastTime = System.nanoTime();

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setF(double kF) {
        this.kF = kF;
    }

    public void setIntegralLimit(double limit) {
        this.integralLimit = Math.abs(limit);
    }

    public void setOutputLimit(double limit) {
        this.outputLimit = Math.abs(limit);
    }

    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        lastTime = System.nanoTime();
    }

    public double calculate(double measurement) {
        long now = System.nanoTime();
        double deltaTime = (now - lastTime) / 1e9;
        lastTime = now;

        if (deltaTime <= 0) {
            return 0;
        }

        double error = setpoint - measurement;

        integralSum += error * deltaTime;
        integralSum = Range.clip(integralSum, -integralLimit, integralLimit);

        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        double output =
                (kP * error) +
                        (kI * integralSum) +
                        (kD * derivative) +
                        (kF * setpoint);

        return Range.clip(output, -outputLimit, outputLimit);
    }
}

