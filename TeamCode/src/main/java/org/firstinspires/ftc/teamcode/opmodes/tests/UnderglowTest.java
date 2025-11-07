package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.TEST)
public class UnderglowTest extends OpMode {

    Underglow underglow;

    Underglow.Color color = null;
    Underglow.Color[] colors = {null, Underglow.Color.ALLIANCE, Underglow.Color.YELLOW, Underglow.Color.GREEN, Underglow.Color.WHITE, Underglow.Color.RAINBOW};
    int colorIndex = 0;

    public void init() {
        underglow = new Underglow(this);
        callTelemetry();
    }

    public void loop() {
        if (gamepad1.bWasPressed()) Global.alliance = Global.Alliance.RED;
        if (gamepad1.xWasPressed()) Global.alliance = Global.Alliance.BLUE;
        if (gamepad1.yWasPressed()) Global.alliance = null;
        if (gamepad1.dpadUpWasPressed()) colorIndex++;
        if (gamepad1.dpadDownWasPressed()) colorIndex--;

        if (colorIndex < 0) colorIndex = colors.length - 1;
        if (colorIndex >= colors.length) colorIndex = 0;
        color = colors[colorIndex];
        underglow.setColor(color);
        callTelemetry();
    }

    public void callTelemetry() {
        telemetry.addLine("This program will cycle underglow colors based on gamepad input:");
        telemetry.addData("B", "Set Alliance RED");
        telemetry.addData("X", "Set Alliance BLUE");
        telemetry.addData("Y", "Set Alliance NULL (OFF)");
        telemetry.addData("D-PAD UP(+)/DOWN(-)", "Cycle through NULL, ALLIANCE COLOR, YELLOW, GREEN, WHITE, and RAINBOW");
        telemetry.addLine();
        telemetry.addData("Current Color", color == null ? "NULL" : color);
        telemetry.addData("Current Alliance", Global.alliance == null ? "NULL" : Global.alliance);
        telemetry.update();
    }
}
