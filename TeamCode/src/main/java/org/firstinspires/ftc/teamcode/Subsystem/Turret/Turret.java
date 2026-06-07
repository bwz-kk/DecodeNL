package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a recorded (distance, velocity) calibration point for the flywheel lookup table.
 */
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

    /** Tuning: set to non-zero to manually drive the turret motor (bypasses PID). */
    public static double manualTurretPower = 0.0;

    /** Tuning: command turret to a specific angle in degrees. Overrides auto-aim when non-zero. */
    public static double tuningTurretAngleDeg = 0.0;

    /** Flywheel PIDF coefficients — tune live from FTC Dashboard. */
    public static double flywheelP = 0.0;
    public static double flywheelI = 0.0;
    public static double flywheelD = 0.0;
    public static double flywheelF = 0.0;

    private final DcMotorEx turret;

    /** Shooter flywheels */
    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;

    private final InterpLUT velocityInterpolation = new InterpLUT();

    private Pose lastPose       = new Pose(0, 0, 0);
    private Pose botPose        = new Pose(0, 0, 0);
    private Pose poseToAim      = new Pose(0, 0, 0);
    private Pose virtualBotPose = new Pose(0, 0, 0);

    private Vector movementVector     = new Vector(0, 0);
    private final double virtualBotMultiplier = 2.0;

    private double distance      = 0.0;
    private double targetAngleFC = 0.0;
    private int    targetVelocity = 0;

    public PIDController turretController = new PIDController(0, 0, 0);

    private TurretConstants.SIDES side = TurretConstants.SIDES.BLUE;

    private final Telemetry telemetry;

    private static final double MIN_DISTANCE = 24.0;
    private static final double MAX_DISTANCE = 144.0;

    // ── Velocity Calibration Recording ──────────────────────────────────────────

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
        if (calibrationPoints.isEmpty()) return "// No calibration points recorded.";
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

        // Shooter flywheels
        shooter1 = hardwareMap.get(DcMotorEx.class, TurretConstants.HMShooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class, TurretConstants.HMShooter2);

        reinitMotors();
        buildVelocityTable();
    }

    // ── Configuration ──────────────────────────────────────────────────────────

    public void setSide(TurretConstants.SIDES side) {
        this.side = side;
    }

    // ── Pose Updates ───────────────────────────────────────────────────────────

    public void updateBotPose(Pose pose) {
        this.lastPose = this.botPose;
        this.botPose  = pose;

        movementVector = new Vector(
                lastPose.distanceFrom(botPose),
                Math.atan2(
                        botPose.getY() - lastPose.getY(),
                        botPose.getX() - lastPose.getX()
                )
        ).times(virtualBotMultiplier);

        virtualBotPose = new Pose(
                botPose.getX() + movementVector.getXComponent(),
                botPose.getY() + movementVector.getYComponent(),
                botPose.getHeading()
        );
    }

   
    public double getTurretAngle() {
        int ticks       = turret.getCurrentPosition();
        double radians  = (ticks / TurretConstants.ENCODER_CPR) * 2.0 * Math.PI;
        radians        /= TurretConstants.TURRET_GEAR_RATIO;
        return normalizeAngle(radians + TurretConstants.turretEncoderOffset);
    }

    public int getTurretEncoderTicks() {
        return turret.getCurrentPosition();
    }

    public double getDistance() {
        return distance;
    }
    public boolean atVelocity() {
        final int TOLERANCE = 20; // ticks/sec
        return Math.abs(targetVelocity - shooter1.getVelocity()) < TOLERANCE
                && Math.abs(targetVelocity - shooter2.getVelocity()) < TOLERANCE;
    }

    public void setShooterVelocity(int velocity) {
        targetVelocity = velocity;
        shooter1.setVelocity(velocity);
        shooter2.setVelocity(velocity);
    }

    public void reinitMotors() {
     
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(0, 0, 0, 0));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(0, 0, 0, 0));
    }

   

    @Override
    public void periodic() {
        poseToAim = TurretConstants.getGoalPose(side);

        updateTurret();
        updateShooter();
        reportTelemetry();
    }

    private void updateTurret() {
        // Manual turret override for tuning — bypasses PID entirely
        if (manualTurretPower != 0.0) {
            double power = Range.clip(manualTurretPower, -0.7, 0.7);
            turret.setPower(power);
            distance = botPose.distanceFrom(poseToAim);
            return;
        }

        // Tuning: command turret to a specific angle in degrees
        if (tuningTurretAngleDeg != 0.0) {
            double targetRad = Math.toRadians(tuningTurretAngleDeg);
            targetRad = Range.clip(targetRad,
                    -TurretConstants.TURRET_SOFT_LIMIT_RADIANS,
                    TurretConstants.TURRET_SOFT_LIMIT_RADIANS);
            double currentAngle = getTurretAngle();
            double rawPower = turretController.calculate(currentAngle, targetRad) / 2.0;
            double power = Range.clip(rawPower, -0.7, 0.7);
            turret.setPower(power);
            distance = botPose.distanceFrom(poseToAim);
            return;
        }

        targetAngleFC = -Math.atan2(
                poseToAim.getY() - botPose.getY(),
                poseToAim.getX() - botPose.getX()
        ) + Math.PI;

        double targetAngleRC = normalizeAngle(targetAngleFC + botPose.getHeading());

        if (side == TurretConstants.SIDES.RED) {
            targetAngleRC += TurretConstants.redOffset;
        } else {
            targetAngleRC += TurretConstants.blueOffset;
        }

        targetAngleRC = Range.clip(
                targetAngleRC,
                -TurretConstants.TURRET_SOFT_LIMIT_RADIANS,
                TurretConstants.TURRET_SOFT_LIMIT_RADIANS
        );

        double currentAngle = getTurretAngle();
        double rawPower = turretController.calculate(currentAngle, targetAngleRC) / 2.0;

        boolean pastPositiveLimit = currentAngle >  TurretConstants.TURRET_HARD_LIMIT_RADIANS;
        boolean pastNegativeLimit = currentAngle < -TurretConstants.TURRET_HARD_LIMIT_RADIANS;
        if (pastPositiveLimit && rawPower > 0) rawPower = 0;
        if (pastNegativeLimit && rawPower < 0) rawPower = 0;

        double power = Range.clip(rawPower, -0.7, 0.7);
        turret.setPower(power);

        distance = botPose.distanceFrom(poseToAim);
    }

    private void updateShooter() {
        // Apply flywheel PIDF from Dashboard if any coefficient is non-zero
        if (flywheelP != 0.0 || flywheelI != 0.0 || flywheelD != 0.0 || flywheelF != 0.0) {
            PIDFCoefficients coeffs = new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF);
            shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
            shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        }

        int desiredVelocity;
        if (tuningVelocity > 0) {
            desiredVelocity = tuningVelocity;
        } else {
            desiredVelocity = (int) velocityInterpolation.get(
                    Range.clip(distance, MIN_DISTANCE + 1, MAX_DISTANCE - 1)
            );
        }
        setShooterVelocity(desiredVelocity);
    }

    private void reportTelemetry() {
        telemetry.addData("[Turret] Angle (rad)",       getTurretAngle());
        telemetry.addData("[Turret] Angle (deg)",       Math.toDegrees(getTurretAngle()));
        telemetry.addData("[Turret] Encoder Ticks",     getTurretEncoderTicks());
        telemetry.addData("[Turret] Target Angle RC",   targetAngleFC + botPose.getHeading());
        telemetry.addData("[Turret] Tuning Angle Deg",  tuningTurretAngleDeg);
        telemetry.addData("[Turret] At Soft Limit",
                Math.abs(getTurretAngle()) >= TurretConstants.TURRET_SOFT_LIMIT_RADIANS);
        telemetry.addData("[Turret] At Hard Limit",
                Math.abs(getTurretAngle()) >= TurretConstants.TURRET_HARD_LIMIT_RADIANS);
        telemetry.addData("[Turret] Distance",          distance);
        telemetry.addData("[Turret] Manual Power",      manualTurretPower);
        telemetry.addData("[Turret] Target Velocity",   targetVelocity);
        telemetry.addData("[Turret] Shooter1 Velocity", shooter1.getVelocity());
        telemetry.addData("[Turret] Shooter2 Velocity", shooter2.getVelocity());
        telemetry.addData("[Turret] At Velocity",       atVelocity());
        telemetry.addData("[Turret] Bot Pose",          botPose);
        telemetry.addData("[Turret] Virtual Bot Pose",  virtualBotPose);
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void buildVelocityTable() {
        velocityInterpolation.add(MIN_DISTANCE,      0);
        velocityInterpolation.add(MIN_DISTANCE + 1,  0);
        velocityInterpolation.add(MAX_DISTANCE - 1,  0);
        velocityInterpolation.add(MAX_DISTANCE,      0);
        velocityInterpolation.createLUT();
    }
}