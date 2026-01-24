package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "Set Alliance Blue", group = Global.OpModeGroup.SET_ALLIANCE)
public class SetAllianceBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Alliance set to BLUE.\nPress START to enable underglow\nPress STOP to disable underglow");
        telemetry.update();
        Global.alliance = null;
        Underglow underglow = new Underglow(this);
        waitForStart();
        if (opModeIsActive()) {
            Global.alliance = Global.Alliance.BLUE;
            underglow.enable(3);
        }
        requestOpModeStop();
    }
}