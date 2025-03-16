package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "Set Alliance Blue", group = "!set alliance")
public class SetAllianceBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Press START to set alliance to BLUE, or press STOP now to disable");
        telemetry.update();
        Global.alliance = null;
        waitForStart();
        if (opModeIsActive()) {
            Global.alliance = Global.Alliance.BLUE;
        }
        requestOpModeStop();
    }
}