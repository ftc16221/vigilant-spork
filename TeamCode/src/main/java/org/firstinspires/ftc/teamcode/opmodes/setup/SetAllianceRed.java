package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Indicator;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "Set Alliance Red", group = Global.OpModeGroup.SETUP)
public class SetAllianceRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Alliance set to RED.\nPress START to enable underglow\nPress STOP to disable underglow");
        telemetry.update();
        Indicator.enabled = false;
        waitForStart();
        if (opModeIsActive()) {
            Indicator.enabled = true;
            Global.alliance = Global.Alliance.RED;
        }
        requestOpModeStop();
    }
}