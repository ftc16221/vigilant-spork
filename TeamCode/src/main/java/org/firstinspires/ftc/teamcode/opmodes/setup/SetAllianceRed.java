package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "Set Alliance Red", group = Global.OpModeGroup.SETUP)
public class SetAllianceRed extends SetAlliance {
    @Override
    public void setAllianceColor() {
        setAlliance(Global.Alliance.RED);
        super.setAllianceColor();
    }
}