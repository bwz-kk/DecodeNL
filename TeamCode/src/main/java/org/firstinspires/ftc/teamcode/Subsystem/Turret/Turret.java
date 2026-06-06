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

@Config
public class Turret extends SubsystemBase {

    public static int tuningVelocity = 0;

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

    public Turret(HardwareMap hardwareMap) {
        telemetry = FtcDashboard.getInstance().getTelemetry();

        // Turret motor — the REV Through Bore Encoder V1 is wired as a quadrature
        // encoder on this motor port, so turret.getCurrentPosition() reads encoder
        // ticks directly.  The internal motor encoder is overridden by the external
        // Through Bore Encoder's A/B channels.
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

    // ── Encoder-to-Angle Conversion ────────────────────────────────────────────
    //
    //   turretAngle_rad = (encoderTicks / ENCODER_CPR) × 2π / TURRET_GEAR_RATIO
    //                     + turretEncoderOffset
    //
    // ZEROING STRATEGY
    //   The encoder is zeroed (STOP_AND_RESET_ENCODER) in reinitMotors(), called
    //   during construction.  This means "zero" is defined as the turret position
    //   at robot init / OpMode start.  The turretEncoderOffset constant is tuned
    //   so that getTurretAngle() reports 0 when the turret physically points at
    //   the reference direction (typically straight ahead / toward the field
    //   centre-line).
    //
    //   If the turret is not at the physical zero at init time, the offset will
    //   still make the math work for aiming — but the full ±130° range must be
    //   reachable.  Verify on the robot.
    //
    // LIMIT PROTECTION
    //   Hard-stop protection prevents the motor from driving further into a
    //   mechanical stop when the angle is already past TURRET_HARD_LIMIT_RADIANS.
    //   The PID output is only clamped in the direction that would worsen the
    //   over-travel; it is still allowed in the escape direction.

    /**
     * Returns the current turret angle in radians, normalized to [-π, π].
     * Reads the quadrature encoder ticks from the turret motor port and converts
     * to radians using ENCODER_CPR, TURRET_GEAR_RATIO, and turretEncoderOffset.
     */
    public double getTurretAngle() {
        int ticks       = turret.getCurrentPosition();
        double radians  = (ticks / TurretConstants.ENCODER_CPR) * 2.0 * Math.PI;
        radians        /= TurretConstants.TURRET_GEAR_RATIO;
        return normalizeAngle(radians + TurretConstants.turretEncoderOffset);
    }

    /**
     * Returns the raw encoder tick count from the turret motor port.
     * Useful for calibrating turretEncoderOffset.
     */
    public int getTurretEncoderTicks() {
        return turret.getCurrentPosition();
    }

    /**
     * Returns distance (inches) from the robot's current pose to the target goal.
     */
    public double getDistance() {
        return distance;
    }

    // ── Shooter ────────────────────────────────────────────────────────────────

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

    // ── Initialization ─────────────────────────────────────────────────────────

    public void reinitMotors() {
        // Stop and reset the turret motor encoder.  Because the REV Through Bore
        // Encoder V1 is wired to this motor port, the encoder ticks reflect the
        // external encoder, not the internal motor encoder.  This establishes the
        // zero position for getTurretAngle() at the turret's physical position
        // when init() is called.
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

    // ── SubsystemBase ──────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        poseToAim = TurretConstants.getGoalPose(side);

        updateTurret();
        updateShooter();
        reportTelemetry();
    }

    private void updateTurret() {
        targetAngleFC = -Math.atan2(
                poseToAim.getY() - botPose.getY(),
                poseToAim.getX() - botPose.getX()
        ) + Math.PI;

        double targetAngleRC = normalizeAngle(targetAngleFC + botPose.getHeading());

        // Apply alliance-specific mechanical offset
        if (side == TurretConstants.SIDES.RED) {
            targetAngleRC += TurretConstants.redOffset;
        } else {
            targetAngleRC += TurretConstants.blueOffset;
        }

        // Clamp target to soft limits (±128° — 2° margin inside the ±130° mechanical hard stops)
        targetAngleRC = Range.clip(
                targetAngleRC,
                -TurretConstants.TURRET_SOFT_LIMIT_RADIANS,
                TurretConstants.TURRET_SOFT_LIMIT_RADIANS
        );

        // Read current angle from encoder and close the loop
        double currentAngle = getTurretAngle();
        double rawPower = turretController.calculate(currentAngle, targetAngleRC) / 2.0;

        // Hard-stop protection: if the encoder already reads past the mechanical limit,
        // prevent the motor from driving further in that direction.
        // (PID output is still allowed in the escape direction.)
        boolean pastPositiveLimit = currentAngle >  TurretConstants.TURRET_HARD_LIMIT_RADIANS;
        boolean pastNegativeLimit = currentAngle < -TurretConstants.TURRET_HARD_LIMIT_RADIANS;
        if (pastPositiveLimit && rawPower > 0) rawPower = 0;
        if (pastNegativeLimit && rawPower < 0) rawPower = 0;

        double power = Range.clip(rawPower, -0.7, 0.7);
        turret.setPower(power);

        distance = botPose.distanceFrom(poseToAim);
    }

    private void updateShooter() {
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
        telemetry.addData("[Turret] At Soft Limit",
                Math.abs(getTurretAngle()) >= TurretConstants.TURRET_SOFT_LIMIT_RADIANS);
        telemetry.addData("[Turret] At Hard Limit",
                Math.abs(getTurretAngle()) >= TurretConstants.TURRET_HARD_LIMIT_RADIANS);
        telemetry.addData("[Turret] Distance",          distance);
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