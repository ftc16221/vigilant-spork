package org.firstinspires.ftc.teamcode.opmodes.tests;

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
        spindexer.update();
        launcher.update();
        spindexer.runTelemetry();
    }
}
