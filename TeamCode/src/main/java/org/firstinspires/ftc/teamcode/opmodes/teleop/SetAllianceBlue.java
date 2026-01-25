package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "Set Alliance Blue", group = Global.OpModeGroup.SET_ALLIANCE)
public class SetAllianceBlue extends SetAlliance {
    @Override
    public void setAllianceColor() {
        setAlliance(Global.Alliance.BLUE);
        super.setAllianceColor();
    }
}