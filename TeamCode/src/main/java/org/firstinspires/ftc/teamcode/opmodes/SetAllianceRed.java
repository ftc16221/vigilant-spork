package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "Set Alliance Red", group = "Set Alliance")
public class SetAllianceRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Press START to set alliance to RED");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            Global.alliance = Global.Alliance.RED;
        }
        requestOpModeStop();
    }
}