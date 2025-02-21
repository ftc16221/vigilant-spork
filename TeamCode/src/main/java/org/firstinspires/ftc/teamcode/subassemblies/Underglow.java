package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Subassembly;

public class Underglow extends Subassembly {

    RevBlinkinLedDriver underglow;

    public Underglow(OpMode opMode) {
        super(opMode,"Underglow");
        underglow = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "underglow");
        setAlliance(Global.alliance);
    }

    public enum Alliance {
        RED,
        BLUE,
        OFF
    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case RED:
                underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                break;
            case BLUE:
                underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                break;
            case OFF:
                underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                break;
        }
    }
}
