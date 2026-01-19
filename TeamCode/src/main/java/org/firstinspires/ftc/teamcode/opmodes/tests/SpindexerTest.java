package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Launcher;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.TEST)
public class SpindexerTest extends OpMode {

    Intake intake;
    Spindexer spindexer;
    Launcher launcher;

    NormalizedColorSensor colorSensor;

    @Override
    public void init() {
        intake = new Intake(this);
        spindexer = new Spindexer(this, intake);
        launcher = new Launcher(this, spindexer);

        colorSensor = spindexer.getColorSensor();
    }

    @Override
    public void loop() {

        if (gamepad2.bWasPressed()) {
            spindexer.emptyActiveSlot();
        } else if (gamepad2.yWasPressed()) {
            spindexer.alignAnyForLaunch();
        } else if (gamepad2.aWasPressed()) {
            spindexer.alignForIntake();
        }

        if (gamepad2.dpad_up) {
            Spindexer.COLOR_SENSOR_GAIN += 0.1f;
            colorSensor.setGain(Spindexer.COLOR_SENSOR_GAIN);
        } else if (gamepad2.dpad_down) {
            Spindexer.COLOR_SENSOR_GAIN -= 0.1f;
            colorSensor.setGain(Spindexer.COLOR_SENSOR_GAIN);
        }
        telemetry.addData("sensor gain", Spindexer.COLOR_SENSOR_GAIN);

        float[] hsvValues = new float[3];
        NormalizedRGBA colors = spindexer.getColorSensor().getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);


        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2])
                .addData("Alpha", "%.3f", colors.alpha);

        float normRed = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue = colors.blue / colors.alpha;
        float sum = normRed + normGreen + normBlue;
        float redPercent = normRed / sum * 100;
        float greenPercent = normGreen / sum * 100;
        float bluePercent = normBlue / sum * 100;
        telemetry.addLine("normalized color sensor output: ")
                .addData("nR", "%.3f", normRed)
                .addData("nG", "%.3f", normGreen)
                .addData("nB", "%.3f", normBlue)
                .addData("nA", "%.3f", colors.alpha);
        telemetry.addLine("color percentages: ")
                .addData("pR", "%.3f", redPercent)
                .addData("pG", "%.3f", greenPercent)
                .addData("pB", "%.3f", bluePercent);

        spindexer.update();
        launcher.update();
        spindexer.runTelemetry();
    }
}
