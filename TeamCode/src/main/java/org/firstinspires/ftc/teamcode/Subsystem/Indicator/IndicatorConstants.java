package org.firstinspires.ftc.teamcode.Subsystem.Indicator;

public class IndicatorConstants {
    public static final String HMIndicatorLight1 = "indicator_light1";
    public enum Color {
        OFF(0.000),
        RED(0.279),
        YELLOW(0.388),
        GREEN(0.500),
        BLUE(0.611),
        PURPLE(0.722),
        WHITE(1.000);

        public final double position;

        Color(double position) {
            this.position = position;
        }
    }
}
