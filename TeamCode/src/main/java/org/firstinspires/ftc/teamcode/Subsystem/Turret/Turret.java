package org.firstinspires.ftc.teamcode.Subsystem.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.Hardware.TauraServo;

@Config
public class Turret extends SubsystemBase {

    public static int tuningVelocity = 0;

    TauraServo hood;
    TauraServo turretServo1;
    TauraServo turretServo2;
    TauraServo turretServo3;
    DcMotorEx throughBore;
    DcMotorEx shooter1;
    DcMotorEx shooter2;

    InterpLUT velocityInterpolation = new InterpLUT();
    InterpLUT hoodInterpolation = new InterpLUT();

    double minDistance = 0;
    double maxDistance = 0;

    Pose lastPose = new Pose(0, 0, 0);
    Pose botPose = new Pose(0, 0, 0);
    Pose poseToAim = new Pose(0, 0, 0);
    Pose virtualBotPose = new Pose(0, 0, 0);

    private Vector movementVector = new Vector(0, 0);
    double virtualBotMultiplier = 2;

    double distance = 0;
    double targetAngleFC = 0;

    public PIDController turretController = new PIDController(0, 0, 0);
    TurretConstants.SIDES side = TurretConstants.SIDES.BLUE;

    Telemetry telemetry;

    private double turretServoPosition = 0.5;

    public Turret(HardwareMap hardwareMap) {

        telemetry = FtcDashboard.getInstance().getTelemetry();

        hood = new TauraServo(hardwareMap.get(Servo.class, TurretConstants.HMHood));

        turretServo1 = new TauraServo(hardwareMap.get(Servo.class, TurretConstants.TauraServo1));
        turretServo2 = new TauraServo(hardwareMap.get(Servo.class, TurretConstants.TauraServo2));
        turretServo3 = new TauraServo(hardwareMap.get(Servo.class, TurretConstants.TauraServo3));

        throughBore = hardwareMap.get(DcMotorEx.class, TurretConstants.HMThroughBore);
        throughBore.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        throughBore.setPower(0);

        shooter1 = hardwareMap.get(DcMotorEx.class, TurretConstants.HMShooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class, TurretConstants.HMShooter2);
        reinitMotors();

        velocityInterpolation.add(minDistance, 0);
        velocityInterpolation.add(0, 0);
        velocityInterpolation.add(0, 0);
        velocityInterpolation.add(0, 0);
        velocityInterpolation.add(maxDistance, 0);
        velocityInterpolation.createLUT();

        hoodInterpolation.add(minDistance, 0);
        hoodInterpolation.add(0, 0);
        hoodInterpolation.add(maxDistance, 0);
        hoodInterpolation.createLUT();
    }

    public void setSide(TurretConstants.SIDES side) {
        this.side = side;
    }

    public double getDistance() {
        return distance;
    }

    public void zeroTurret() {
        TurretConstants.turretZeroOffsetTicks = throughBore.getCurrentPosition();
    }

    public double getTurretAngle() {
        long ticks = throughBore.getCurrentPosition();
        long relativeTicks = ticks - TurretConstants.turretZeroOffsetTicks;
        double angleRadians = relativeTicks * (2.0 * Math.PI) / TurretConstants.THROUGH_BORE_TICKS_PER_REV;
        while (angleRadians > Math.PI)  angleRadians -= 2.0 * Math.PI;
        while (angleRadians < -Math.PI) angleRadians += 2.0 * Math.PI;
        return angleRadians;
    }

    public void updateBotPose(Pose pose) {
        this.lastPose = this.botPose;
        this.botPose = pose;

        movementVector = new Vector(
                lastPose.distanceFrom(botPose),
                Math.atan2(botPose.getY() - lastPose.getY(), botPose.getX() - lastPose.getX())
        ).times(virtualBotMultiplier);

        virtualBotPose = new Pose(
                botPose.getX() + movementVector.getXComponent(),
                botPose.getY() + movementVector.getYComponent(),
                botPose.getHeading()
        );
    }

    public void reinitMotors() {
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0, 0, 0, 0));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0, 0, 0, 0));
    }

    int targetVelocity = 0;

    public void setShooterVelocity(int power) {
        targetVelocity = power;
        shooter1.setVelocity(power);
        shooter2.setVelocity(power);
    }

    @Override
    public void periodic() {
        switch (side) {
            case RED:
                this.poseToAim = TurretConstants.redGoalPose;
                break;
            case BLUE:
                this.poseToAim = TurretConstants.blueGoalPose;
                break;
        }
        updateTurret();
        updateShooter();
        updateHood();

        telemetry.addData("Position: ", getTurretAngle());
        telemetry.addData("Botpose: ", botPose);
        telemetry.addData("Virtual botpose: ", virtualBotPose);
        telemetry.addData("Distance: ", distance);
        telemetry.addData("Encoder Shooter1: ", shooter1.getVelocity());
        telemetry.addData("Encoder Shooter2: ", shooter2.getVelocity());
        telemetry.update();
    }

    public boolean atVelocity() {
        int tolerance = 0;
        return Math.abs(this.targetVelocity - shooter1.getVelocity()) < tolerance;
    }

    private void updateTurret() {
        targetAngleFC = -Math.atan2(poseToAim.getY() - botPose.getY(), poseToAim.getX() - botPose.getX()) + Math.PI;
        double targetAngleRC = normalizeAngle(targetAngleFC + botPose.getHeading());

        if (this.side == TurretConstants.SIDES.RED) {
            targetAngleRC += TurretConstants.redOffset;
        } else {
            targetAngleRC += TurretConstants.blueOffset;
        }

        targetAngleRC = Range.clip(targetAngleRC, TurretConstants.turretMinAngle, TurretConstants.turretMaxAngle);

        double currentAngle = getTurretAngle();


        boolean atMaxAngle = currentAngle >= TurretConstants.turretMaxAngle;
        boolean atMinAngle = currentAngle <= TurretConstants.turretMinAngle;

        double pidOutput = Range.clip(
                turretController.calculate(currentAngle, targetAngleRC) / 2,
                -0.7, 0.7
        );

        if ((atMaxAngle && pidOutput > 0) || (atMinAngle && pidOutput < 0)) {
            pidOutput = 0;
        }

        turretServoPosition = Range.clip(
                turretServoPosition + pidOutput,
                TurretConstants.turretMinPosition,
                TurretConstants.turretMaxPosition
        );

        turretServo1.setPosition(turretServoPosition);
        turretServo2.setPosition(turretServoPosition);
        turretServo3.setPosition(turretServoPosition);

        distance = botPose.distanceFrom(poseToAim);
    }

    private void updateShooter() {
        setShooterVelocity(
                (int) velocityInterpolation.get(Range.clip(distance, minDistance + 1, maxDistance - 1))
        );
    }

    private void updateHood() {
        double position = hoodInterpolation.get(Range.clip(distance, minDistance + 1, maxDistance - 1));
        position = Range.clip(position, TurretConstants.hoodMinPosition, TurretConstants.hoodMaxPosition);
        hood.setPosition(position);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}