package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.SetAlliance;

@SuppressWarnings("unused")
@TeleOp(name = "Set Alliance Blue", group = "Set Alliance")
public class SetAllianceBlue extends SetAlliance {
    SetAllianceBlue() {
        super(Underglow.Alliance.BLUE);
    }
}
