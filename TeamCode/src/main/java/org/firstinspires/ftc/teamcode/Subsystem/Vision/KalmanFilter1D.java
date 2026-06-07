package org.firstinspires.ftc.teamcode.Subsystem.Vision;

public class KalmanFilter1D {

    private double estimate;
    private double estimateError;

    private final double processNoise;
    private final double measurementNoise;

    public KalmanFilter1D(double initialEstimate, double initialError,
                          double processNoise, double measurementNoise) {
        this.estimate        = initialEstimate;
        this.estimateError   = initialError;
        this.processNoise    = processNoise;
        this.measurementNoise = measurementNoise;
    }

    public void predict() {
        estimateError += processNoise;
    }

    public double update(double measurement) {
        double gain = estimateError / (estimateError + measurementNoise);

        estimate      = estimate + gain * (measurement - estimate);
        estimateError = (1.0 - gain) * estimateError;

        return estimate;
    }

    public double getEstimate() { return estimate; }

    public void reset(double value) {
        estimate      = value;
        estimateError = 100.0;
    }
}
