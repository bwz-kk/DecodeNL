package org.firstinspires.ftc.teamcode.Subsystem.Vision;

/**
 * Decoupled 1D Kalman filter.
 * State: scalar position estimate.
 * Measurement: vision position reading.
 */
public class KalmanFilter1D {

    // Estimate and its uncertainty (variance)
    private double estimate;
    private double estimateError;

    // Tuning parameters
    private final double processNoise;    // Q: how much we expect the estimate to drift per update
    private final double measurementNoise; // R: how much we trust the sensor (higher = less trust)

    /**
     * @param initialEstimate     Starting position value
     * @param initialError        Initial uncertainty in the estimate (large = unsure)
     * @param processNoise        Q - how noisy/dynamic the system is (tune up if robot moves fast)
     * @param measurementNoise    R - sensor noise variance (tune up if vision is jittery)
     */
    public KalmanFilter1D(double initialEstimate, double initialError,
                          double processNoise, double measurementNoise) {
        this.estimate        = initialEstimate;
        this.estimateError   = initialError;
        this.processNoise    = processNoise;
        this.measurementNoise = measurementNoise;
    }

    /**
     * Call every loop even without a measurement so uncertainty grows over time.
     */
    public void predict() {
        estimateError += processNoise;
    }

    /**
     * Call when a new vision measurement is available.
     * @param measurement Raw value from the sensor
     * @return Filtered estimate
     */
    public double update(double measurement) {
        // Kalman gain: how much weight to give the new measurement
        double gain = estimateError / (estimateError + measurementNoise);

        estimate      = estimate + gain * (measurement - estimate);
        estimateError = (1.0 - gain) * estimateError;

        return estimate;
    }

    public double getEstimate() { return estimate; }

    public void reset(double value) {
        estimate      = value;
        estimateError = 100.0; // treat as freshly uncertain after a hard reset
    }
}