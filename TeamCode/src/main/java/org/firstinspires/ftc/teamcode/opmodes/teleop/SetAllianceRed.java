package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "Set Alliance Red", group = "!set alliance")
public class SetAllianceRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Press START to set alliance to RED, or press STOP now to disable");
        telemetry.update();
        Global.alliance = null;
        waitForStart();
        if (opModeIsActive()) {
            Global.alliance = Global.Alliance.RED;
        }
        requestOpModeStop();
    }
}