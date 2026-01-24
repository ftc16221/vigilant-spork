package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.ColorNameLookup;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.TEST)
public class UnderglowTest extends OpMode {

    Underglow underglow;

    int color = Color.BLACK;
    int[] colors = {Color.BLACK, -1, Color.YELLOW, Color.GREEN, Color.CYAN, Color.MAGENTA, Color.WHITE,
            ColorNameLookup.getColorByName("Lavender").getIntValue()};
    int colorIndex = 0;
    int brightness = 1;
    int stickIndex = 0;
    int ledIndex = -1;




    boolean dpadWasPressed = false;
    private Telemetry.Item telemetryColor;
    private Telemetry.Item telemetryAlliance;
    private Telemetry.Item telemetryEnabled;
    private Telemetry.Item telemetryBrightness;
    private Telemetry.Item telemetryLEDIndex;

    public void init() {
        underglow = new Underglow(this);
        underglow.enable(brightness);
        callTelemetry();
    }

    public void loop() {
        if (gamepad1.a) {
            // set all LEDs at once
            ledIndex = -1;
            underglow.enable(brightness);
        }
        if (gamepad1.b) {
            Global.alliance = Global.Alliance.RED;
            colors[1] = Color.RED;
            underglow.setColor(Color.RED);
        }
        if (gamepad1.x) {
            Global.alliance = Global.Alliance.BLUE;
            colors[1] = Color.BLUE;
            underglow.setColor(Color.BLUE);
        }
        if (gamepad1.y) {
            Global.alliance = null;
            colors[1] = Color.BLACK;
            underglow.setColor(Color.BLACK);
        }
        if (gamepad1.dpadUpWasReleased()) {
            if (colorIndex < colors.length - 1) colorIndex++;
            underglow.setColor(stickIndex, ledIndex, colors[colorIndex]);
        }
        if (gamepad1.dpadDownWasReleased()) {
            if (colorIndex > 0) colorIndex--;
            underglow.setColor(stickIndex, ledIndex, colors[colorIndex]);
        }
        if (gamepad1.dpadRightWasReleased()) {
            if (ledIndex < 9) ledIndex++;
        }
        if (gamepad1.dpadLeftWasReleased()) {
            if (ledIndex > 0) ledIndex--;
        }
        if (gamepad1.rightBumperWasReleased()) {
            if (brightness < 30) brightness++;
            underglow.setBrightness(brightness);
        }
        if (gamepad1.leftBumperWasReleased()) {
            if (brightness > 0) brightness--;
            underglow.setBrightness(brightness);
        }

        if (colorIndex < 0) colorIndex = colors.length - 1;
        if (colorIndex >= colors.length) colorIndex = 0;
        color = colors[colorIndex];

        callTelemetry();
    }


    public void callTelemetry() {

        telemetry.addLine("This program will cycle underglow colors based on gamepad input:");
        telemetry.addData("A", "Set All LEDs at once");
        telemetry.addData("B", "Set Alliance RED");
        telemetry.addData("X", "Set Alliance BLUE");
        telemetry.addData("Y", "Set Alliance NULL (OFF)");
        telemetry.addLine("D-PAD UP(+)/DOWN(-) to Cycle through BLACK (off), ALLIANCE COLOR, YELLOW, GREEN, CYAN, MAGENTA, and WHITE");
        if (colorIndex >= (colors.length - 1)) {
            telemetry.addData("*  D-PAD UP (at Maximum)", ColorNameLookup.getColorByInt(color).getName());
        } else {
            telemetry.addData("*  D-PAD UP", ColorNameLookup.getColorByInt(colors[colorIndex + 1]).getName());
        }
        if (colorIndex < 1) {
            telemetry.addData("*  D-PAD DOWN (at Minimum)", ColorNameLookup.getColorByInt(color).getName());
        } else {
            telemetry.addData("*  D-PAD DOWN", ColorNameLookup.getColorByInt(colors[colorIndex - 1]).getName());
        }
        telemetry.addLine("D-PAD RIGHT(+)/LEFT(-) to Increment/Decrement LED Index");
        telemetry.addData("*  D-PAD RIGHT", ledIndex + 1);
        telemetry.addData("*  D-PAD LEFT", ledIndex - 1);
        telemetry.addLine("Bumpers RIGHT(+)/LEFT(-) to Increment/Decrement Brightness");
        telemetry.addData("*  Bumper RIGHT", brightness + 1);
        telemetry.addData("*  Bumper LEFT", brightness -1);

        telemetry.addLine();
        telemetry.addData("Underglow Enabled", underglow.isEnabled());
        telemetry.addData("Current Color", ColorNameLookup.getColorByInt(color).getName());
        telemetry.addData("Current Alliance", Global.alliance == null ? "NULL" : Global.alliance);
        telemetry.addData("Current LED Index", ledIndex);
        telemetry.addData("Current Brightness", brightness);

        telemetry.update();
    }
//    public void callTelemetry() {
//        telemetryColor.setValue(colors[colorIndex]);
//        telemetryAlliance.setValue(Global.alliance == null ? "NULL" : Global.alliance);
//        telemetryEnabled.setValue(underglow.isEnabled());
//        telemetryBrightness.setValue(brightness);
//        telemetry.update();
//    }
}
