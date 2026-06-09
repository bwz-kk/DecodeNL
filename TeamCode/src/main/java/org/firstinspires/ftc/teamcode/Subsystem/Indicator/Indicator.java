package org.firstinspires.ftc.teamcode.Subsystem.Indicator;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * RGB Indicator Light subsystem wrapping the GoBilda RGB Indicator Light
 * Provides simple color control through servo position mapping
 */
@Config
public class Indicator extends SubsystemBase {
    private Servo indicatorServo1;
    private IndicatorConstants.Color currentColor = IndicatorConstants.Color.OFF;

    public Indicator(HardwareMap hardwareMap) {
        indicatorServo1 = hardwareMap.get(Servo.class, IndicatorConstants.HMIndicatorLight1);
        setColor(IndicatorConstants.Color.OFF);
    }

    public void setColor(IndicatorConstants.Color color) {
        currentColor = color;
        indicatorServo1.setPosition(color.position);
    }
    public IndicatorConstants.Color getColor() {
        return currentColor;
    }

    @Override
    public void periodic() {
    }
}
