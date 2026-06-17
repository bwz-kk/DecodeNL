package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Commands.Indicator.SetIndicatorColor;
import org.firstinspires.ftc.teamcode.Subsystem.Indicator.Indicator;
import org.firstinspires.ftc.teamcode.Subsystem.Indicator.IndicatorConstants;


public class IndicatorLightTeleOpExample extends CommandOpMode {
    private Indicator indicator1;

    @Override
    public void initialize() {
        indicator1 = new Indicator(hardwareMap);

        indicator1.setColor(IndicatorConstants.Color.OFF);

        super.reset();

        telemetry.addLine("Indicator Light Controls:");
        telemetry.addLine("A: RED | B: YELLOW | X: GREEN | Y: BLUE");
        telemetry.addLine("Left Bumper: PURPLE | Right Bumper: WHITE | DPAD Down: OFF");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        if (gamepad1.a) {
            schedule(new SetIndicatorColor(indicator1, IndicatorConstants.Color.RED));
        } else if (gamepad1.b) {
            schedule(new SetIndicatorColor(indicator1, IndicatorConstants.Color.YELLOW));
        } else if (gamepad1.x) {
            schedule(new SetIndicatorColor(indicator1, IndicatorConstants.Color.GREEN));
        } else if (gamepad1.y) {
            schedule(new SetIndicatorColor(indicator1, IndicatorConstants.Color.BLUE));
        } else if (gamepad1.left_bumper) {
            schedule(new SetIndicatorColor(indicator1, IndicatorConstants.Color.PURPLE));
        } else if (gamepad1.right_bumper) {
            schedule(new SetIndicatorColor(indicator1, IndicatorConstants.Color.WHITE));
        } else if (gamepad1.dpad_down) {
            schedule(new SetIndicatorColor(indicator1, IndicatorConstants.Color.OFF));
        }

        telemetry.addData("Current Color1", indicator1.getColor());
        telemetry.addData("Servo Position1", indicator1.getColor().position);
        telemetry.update();
    }
}
