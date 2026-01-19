package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Launcher;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.TEST)
public class SpindexerTest extends OpMode {

    Intake intake;
    Spindexer spindexer;
    Launcher launcher;

    @Override
    public void init() {
        intake = new Intake(this);
        spindexer = new Spindexer(this, intake);
        launcher = new Launcher(this, spindexer);
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

        float[] hsvValues = new float[3];
        int color = spindexer.getColorSensor().getNormalizedColors().toColor();
        Color.colorToHSV(color, hsvValues);

        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);

        spindexer.update();
        launcher.update();
        spindexer.runTelemetry();
    }
}
