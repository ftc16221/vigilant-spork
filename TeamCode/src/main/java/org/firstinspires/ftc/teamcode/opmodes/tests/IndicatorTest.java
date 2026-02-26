package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Indicator;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.TEST)
public class IndicatorTest extends OpMode {

    Indicator indicator;

    int color = Color.BLACK;
    int[] colors = {Color.BLACK, -1, Color.YELLOW, Color.GREEN, Color.WHITE};
    int colorIndex = 0;

    boolean dpadWasPressed = false;

    public void init() {
        indicator = new Indicator(this);
        callTelemetry();
    }

    public void loop() {
        if (gamepad1.b) Global.alliance = Global.Alliance.RED;
        if (gamepad1.x) Global.alliance = Global.Alliance.BLUE;
        if (gamepad1.y) Global.alliance = null;
        if (gamepad1.dpad_up && !dpadWasPressed) colorIndex++;
        if (gamepad1.dpad_down && !dpadWasPressed) colorIndex--;

        dpadWasPressed = gamepad1.dpad_up || gamepad1.dpad_down;

        if (colorIndex < 0) colorIndex = colors.length - 1;
        if (colorIndex >= colors.length) colorIndex = 0;
        color = colors[colorIndex];
        Indicator.setAllColor(color);
        callTelemetry();
    }

    public void callTelemetry() {
        telemetry.addLine("This program will cycle underglow colors based on gamepad input:");
        telemetry.addData("B", "Set Alliance RED");
        telemetry.addData("X", "Set Alliance BLUE");
        telemetry.addData("Y", "Set Alliance NULL (OFF)");
        telemetry.addData("D-PAD UP(+)/DOWN(-)", "Cycle through BLACK, ALLIANCE COLOR, YELLOW, GREEN, and WHITE");
        telemetry.addLine();
        telemetry.addData("Current Color", color);
        telemetry.addData("Current Alliance", Global.alliance == null ? "NULL" : Global.alliance);
        telemetry.update();
    }
}
