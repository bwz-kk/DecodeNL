package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import static org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants.maxShootingDistance;
import static org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants.minShootingDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.Indicator.Indicator;
import org.firstinspires.ftc.teamcode.Subsystem.Indicator.IndicatorConstants;

import java.util.ArrayList;
import java.util.List;

@Config
public class Turret extends SubsystemBase {

    public static double manualAngle = 0.0;
    public static boolean useManualAngle = false;

    public static double kP = 5;
    public static double kI = 0;
    public static double kD = 16;

    public static double ANGLE_STABLE_TOLERANCE_DEG = 1;

    private final DcMotorEx turret;
    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;

    private final Indicator indicator;

    private final InterpLUT velocityInterpolation = new InterpLUT();
    private boolean okToShoot = false;

    private Pose botPose = new Pose(0, 0, 0);

    private double distance = 0.0;

    public PIDController turretController = new PIDController(3, 0, 0.09);

    private TurretConstants.SIDES side = TurretConstants.SIDES.BLUE;
    public static int targetVelocity = 0;

    private final Telemetry telemetry;


    private final List<VelocityCalibrationPoint> calibrationPoints = new ArrayList<>();

    public Turret(HardwareMap hardwareMap, Indicator indicator) {
        this.indicator = indicator;
        telemetry = FtcDashboard.getInstance().getTelemetry();
        turret = hardwareMap.get(DcMotorEx.class, TurretConstants.HMTurret);
        shooter1 = hardwareMap.get(DcMotorEx.class, TurretConstants.HMShooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class, TurretConstants.HMShooter2);
        reinitMotors();

        velocityInterpolation.add(minShootingDistance, 980);
        velocityInterpolation.add(74, 1000);
        velocityInterpolation.add(95, 1200);
        velocityInterpolation.add(119, 1300);
        velocityInterpolation.add(maxShootingDistance, 2000);
        velocityInterpolation.createLUT();
    }

    public void setSide(TurretConstants.SIDES side) {
        this.side = side;
    }

    public double getTurretAngle() {
        int ticks = turret.getCurrentPosition();
        double radians = (ticks / TurretConstants.ENCODER_CPR) * 2.0 * Math.PI;
        radians /= TurretConstants.TURRET_GEAR_RATIO;
        return normalizeAngle(radians - TurretConstants.turretEncoderOffset);
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

    public boolean isStable() {
        double errorDeg = Math.abs(Math.toDegrees(turretController.getPositionError()));
        boolean angleOk = errorDeg < ANGLE_STABLE_TOLERANCE_DEG;

        telemetry.addData("[Turret] Stable - Angle OK", angleOk);
        telemetry.addData("[Turret] Stable - Angle Error (deg)", errorDeg);

        return angleOk;
    }

    public void setShooterVelocity(int velocity) {
        //shooter1.setVelocity(velocity);
        //shooter2.setVelocity(velocity);
        shooter1.setVelocity(targetVelocity);
        shooter1.setVelocity(targetVelocity);
    }

    public void updateBotPose(Pose pose) {
        this.botPose = pose;
    }

    public void reinitMotors() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(110, 0, 30, 16.2));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(110, 0, 30, 16.2));
    }


    @Override
    public void periodic() {
        turretController = new PIDController(kP, kI, kD);

        Pose goalPose = TurretConstants.getGoalPose(side);
        updateTurret(goalPose);
        updateShooter();
        reportTelemetry(goalPose);

        indicator.setColor(isStable() ? IndicatorConstants.Color.GREEN : IndicatorConstants.Color.RED);
    }

    private void updateTurret(Pose goalPose) {
        double goalAngleFC = -Math.atan2(
                goalPose.getY() - botPose.getY(),
                goalPose.getX() - botPose.getX()
        );

        double goalAngleBC = goalAngleFC + botPose.getHeading();

        if (side == TurretConstants.SIDES.RED) {
            goalAngleBC += TurretConstants.redOffset;
        } else {
            goalAngleBC += TurretConstants.blueOffset;
        }

        double goalAngleBCCorrected = Range.clip(
                goalAngleBC,
                -TurretConstants.TURRET_SOFT_LIMIT_RADIANS,
                TurretConstants.TURRET_SOFT_LIMIT_RADIANS
        );

        boolean okToShoot = goalAngleBC == goalAngleBCCorrected;

        if (!okToShoot) {
            goalAngleBCCorrected = 0;
        }

        double currentAngle = getTurretAngle();

        turretController.setSetPoint(-goalAngleBCCorrected);

        double power = turretController.calculate(currentAngle) / 2.0;

        boolean positiveLimit = currentAngle >= TurretConstants.TURRET_HARD_LIMIT_RADIANS;
        boolean negativeLimit = currentAngle <= -TurretConstants.TURRET_HARD_LIMIT_RADIANS;

        if (positiveLimit && power > 0) {
            power = 0;
        }

        if (negativeLimit && power < 0) {
            power = 0;
        }

        turret.setPower(Range.clip(power, -0.5, 0.5));

        distance = botPose.distanceFrom(goalPose);
    }

    private void updateShooter() {
        setShooterVelocity(
                (int) velocityInterpolation.get(Range.clip(distance, minShootingDistance + 1, maxShootingDistance - 1))
        );

    }

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
        telemetry.addData("[Turret] Is Stable", isStable());
        telemetry.addData("[Turret] Use Manual Angle", useManualAngle);
        telemetry.addData("[Turret] Manual Angle (deg)", Math.toDegrees(manualAngle));
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}