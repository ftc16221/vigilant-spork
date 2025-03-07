package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;

public abstract class SetAlliance extends LinearOpMode {
    protected Underglow.Color alliance;

    protected SetAlliance(Underglow.Color color) {
        this.alliance = color;
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("Press START to set alliance to");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            Global.alliance = alliance;
        }
    }

}

