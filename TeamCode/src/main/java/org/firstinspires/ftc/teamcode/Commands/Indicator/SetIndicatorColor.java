package org.firstinspires.ftc.teamcode.Commands.Indicator;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.Indicator.Indicator;
import org.firstinspires.ftc.teamcode.Subsystem.Indicator.IndicatorConstants;

/**
 * Command to set the indicator light to a specific color
 * Completes immediately after setting the color
 */
public class SetIndicatorColor extends CommandBase {
    private final Indicator indicator;
    private final IndicatorConstants.Color color;

    public SetIndicatorColor(Indicator indicator, IndicatorConstants.Color color) {
        this.indicator = indicator;
        this.color = color;
        addRequirements(indicator);
    }

    @Override
    public void initialize() {
        indicator.setColor(color);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
