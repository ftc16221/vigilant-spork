package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "Set Alliance Blue", group = Global.OpModeGroup.SETUP)
public class SetAllianceBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Alliance set to BLUE.\nPress START to enable underglow\nPress STOP to disable underglow");
        telemetry.update();
        Global.alliance = null;
        Underglow.enabled = false;
        waitForStart();
        if (opModeIsActive()) {
            Global.alliance = Global.Alliance.BLUE;
            Underglow.enabled = true;
        }
        requestOpModeStop();
    }
}