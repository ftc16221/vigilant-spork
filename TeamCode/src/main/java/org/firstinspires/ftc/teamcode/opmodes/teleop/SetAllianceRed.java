package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "Set Alliance Red", group = "!set alliance")
public class SetAllianceRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Alliance set to RED.\nPress START to enable underglow\nPress STOP to disable underglow");
        telemetry.update();
        Underglow.enabled = false;
        waitForStart();
        if (opModeIsActive()) {
            Underglow.enabled = true;
            Global.alliance = Global.Alliance.RED;
        }
        requestOpModeStop();
    }
}