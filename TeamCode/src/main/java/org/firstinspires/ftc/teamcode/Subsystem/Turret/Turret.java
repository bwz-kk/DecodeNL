package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

class VelocityCalibrationPoint {
    public final double distance;
    public final int velocity;
    public VelocityCalibrationPoint(double distance, int velocity) {
        this.distance = distance;
        this.velocity = velocity;
    }
}

@Config
public class Turret extends SubsystemBase {

    public static int tuningVelocity = 0;

    // Manual angle control for debugging
    public static double manualAngle = 0.0;  // Manual angle override in radians
    public static boolean useManualAngle = false;  // Toggle manual angle control
    
    // PID tuning via Dashboard
    public static double kP = 3.0;
    public static double kI = 0.0;
    public static double kD = 0.09;

    private final DcMotorEx turret;
    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;

    private final InterpLUT velocityInterpolation = new InterpLUT();

    private Pose botPose = new Pose(0, 0, 0);

    private double distance = 0.0;
    private int targetVelocity = 0;

    public PIDController turretController = new PIDController(3, 0, 0.09);

    private TurretConstants.SIDES side = TurretConstants.SIDES.BLUE;

    private final Telemetry telemetry;

    private final List<VelocityCalibrationPoint> calibrationPoints = new ArrayList<>();

    public void recordCalibrationPoint() {
        calibrationPoints.add(new VelocityCalibrationPoint(distance, targetVelocity));
    }

    public List<VelocityCalibrationPoint> getCalibrationPoints() {
        return calibrationPoints;
    }

    public void clearCalibrationPoints() {
        calibrationPoints.clear();
    }

    public String getCalibrationCode() {
        if (calibrationPoints.isEmpty()) return "";
        StringBuilder sb = new StringBuilder();
        for (VelocityCalibrationPoint p : calibrationPoints) {
            sb.append("velocityInterpolation.add(")
                    .append(String.format("%.1f", p.distance)).append(", ")
                    .append(p.velocity).append(");\n");
        }
        sb.append("velocityInterpolation.createLUT();");
        return sb.toString();
    }

    public String getLastCalibrationSummary() {
        if (calibrationPoints.isEmpty()) return "none";
        VelocityCalibrationPoint last = calibrationPoints.get(calibrationPoints.size() - 1);
        return String.format("dist=%.1f, vel=%d", last.distance, last.velocity);
    }

    public int getCalibrationPointCount() {
        return calibrationPoints.size();
    }

    public Turret(HardwareMap hardwareMap) {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        turret = hardwareMap.get(DcMotorEx.class, TurretConstants.HMTurret);
        shooter1 = hardwareMap.get(DcMotorEx.class, TurretConstants.HMShooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class, TurretConstants.HMShooter2);
        reinitMotors();
        buildVelocityTable();
    }

    public void setSide(TurretConstants.SIDES side) {
        this.side = side;
    }

    /**
     * Calculate turret angle from encoder position.
     * Converts encoder ticks to radians, then applies calibration offset.
     */
    public double getTurretAngle() {
        int ticks = turret.getCurrentPosition();
        double radians = (ticks / TurretConstants.ENCODER_CPR) * 2.0 * Math.PI;
        radians /= TurretConstants.TURRET_GEAR_RATIO;
        return normalizeAngle(radians + TurretConstants.turretEncoderOffset);
    }

    public int getTurretEncoderTicks() {
        return turret.getCurrentPosition();
    }

    public double getDistance() {
        return distance;
    }

    public boolean atVelocity() {
        int tolerance = 50;
        return Math.abs(targetVelocity - shooter1.getVelocity()) < tolerance;
    }

    public void setShooterVelocity(int velocity) {
        targetVelocity = velocity;
        shooter1.setVelocity(velocity);
        shooter2.setVelocity(velocity);
    }

    public void updateBotPose(Pose pose) {
        this.botPose = pose;
    }

    public void reinitMotors() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        // Update PID coefficients from dashboard
        turretController = new PIDController(kP, kI, kD);

        Pose goalPose = TurretConstants.getGoalPose(side);
        updateTurret(goalPose);
        updateShooter();
        reportTelemetry(goalPose);
    }

    /**
     * Update turret angle to point at goal.
     * Handles manual override, calculates target angle in robot-centric frame,
     * applies alliance offset, and enforces hard limits.
     */
    private void updateTurret(Pose goalPose) {
        double targetAngleRC;

        // Check if using manual angle override for debugging
        if (useManualAngle) {
            targetAngleRC = manualAngle;
        } else {
            // Calculate angle to goal in field-centric frame
            double dx = goalPose.getX() - botPose.getX();
            double dy = goalPose.getY() - botPose.getY();
            double angleToGoalFC = Math.atan2(dy, dx);
            
            // Convert to robot-centric frame
            targetAngleRC = normalizeAngle(angleToGoalFC - botPose.getHeading());

            // Apply alliance-specific offset
            if (side == TurretConstants.SIDES.RED) {
                targetAngleRC += TurretConstants.redOffset;
            } else {
                targetAngleRC += TurretConstants.blueOffset;
            }
        }

        // Soft clip to turret range (allows PID to work near limits)
        targetAngleRC = Range.clip(targetAngleRC,
                -Math.toRadians(120), Math.toRadians(120));

        // Get current angle
        double currentAngle = getTurretAngle();
        
        // Find shortest path to target (handles angle wrapping)
        double shortestError = normalizeAngle(targetAngleRC - currentAngle);
        double adjustedTarget = currentAngle + shortestError;

        // Calculate PID power output
        double rawPower = turretController.calculate(currentAngle, adjustedTarget) / 2.0;

        // Hard limit: prevent turret from moving past physical limits
        boolean pastPositiveLimit = currentAngle > Math.toRadians(130);
        boolean pastNegativeLimit = currentAngle < -Math.toRadians(130);
        if (pastPositiveLimit && rawPower > 0) rawPower = 0;
        if (pastNegativeLimit && rawPower < 0) rawPower = 0;

        // Final power output
        double power = Range.clip(rawPower, -0.5, 0.5);
        turret.setPower(power);

        // Update distance to goal
        distance = botPose.distanceFrom(goalPose);
    }

    /**
     * Update shooter flywheel velocity based on distance to goal.
     * If tuningVelocity is set, use that instead (for manual testing).
     */
    private void updateShooter() {
        if (tuningVelocity > 0) {
            // Manual tuning mode: use fixed velocity
            setShooterVelocity(tuningVelocity);
        } else {
            // Automatic mode: interpolate velocity based on distance
            double clippedDistance = Range.clip(
                    distance,
                    TurretConstants.minShootingDistance,
                    TurretConstants.maxShootingDistance
            );
            setShooterVelocity((int) velocityInterpolation.get(clippedDistance));
        }
    }

    /**
     * Report turret state to telemetry.
     */
    private void reportTelemetry(Pose goalPose) {
        double currentAngle = getTurretAngle();
        double error = turretController.getPositionError();
        double targetAngle = currentAngle + error;
        
        telemetry.addData("[Turret] Current Angle (deg)", Math.toDegrees(currentAngle));
        telemetry.addData("[Turret] Target Angle (deg)", Math.toDegrees(targetAngle));
        telemetry.addData("[Turret] Error (deg)", Math.toDegrees(error));
        telemetry.addData("[Turret] Raw Encoder Ticks", getTurretEncoderTicks());
        telemetry.addData("[Turret] Encoder Offset", TurretConstants.turretEncoderOffset);
        telemetry.addData("[Turret] Distance (in)", distance);
        telemetry.addData("[Turret] Goal Pose", goalPose);
        telemetry.addData("[Turret] Robot Pose", botPose);
        telemetry.addData("[Turret] Target Velocity (ticks/s)", targetVelocity);
        telemetry.addData("[Turret] Shooter1 Vel (ticks/s)", shooter1.getVelocity());
        telemetry.addData("[Turret] Shooter2 Vel (ticks/s)", shooter2.getVelocity());
        telemetry.addData("[Turret] At Velocity", atVelocity());
        telemetry.addData("[Turret] Use Manual Angle", useManualAngle);
        telemetry.addData("[Turret] Manual Angle (deg)", Math.toDegrees(manualAngle));
        telemetry.addData("[Calibration] Points Recorded", getCalibrationPointCount());
        telemetry.addData("[Calibration] Last Point", getLastCalibrationSummary());
        telemetry.update();
    }

    /**
     * Normalize angle to [-π, π] range.
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Build velocity interpolation lookup table.
     * CALIBRATION REQUIRED: Replace these placeholder values with real measurements.
     * 
     * Procedure:
     * 1. Place robot at various distances from goal (24", 48", 72", etc.)
     * 2. Set tuningVelocity in Dashboard to a test value
     * 3. Adjust until shooter accurately scores
     * 4. Note the distance and velocity
     * 5. Add them to this table
     * 6. Use getCalibrationCode() to generate the proper table
     */
    private void buildVelocityTable() {
        // Placeholder values - MUST BE CALIBRATED FOR YOUR ROBOT
        velocityInterpolation.add(24, 1200);    // 2 ft minimum range
        velocityInterpolation.add(48, 1500);    // 4 ft
        velocityInterpolation.add(72, 1800);    // 6 ft (typical range)
        velocityInterpolation.add(120, 2200);   // 10 ft
        velocityInterpolation.add(240, 2800);   // 20 ft maximum range
        velocityInterpolation.createLUT();
    }
}
