package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Subassembly;

public class Underglow extends Subassembly {

    RevBlinkinLedDriver underglow;
    private Color lastColor;

    public Underglow(OpMode opMode) {
        super(opMode,"Underglow");
        underglow = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "underglow");
        setColor(Color.ALLIANCE);
    }

    public void setColor(Color color) {
        if (color == lastColor) return; // only set strip color if it has changed
        lastColor = color;

        if (color == null || Global.alliance == null) {
            underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            return;
        }

        switch (color) {
            case ALLIANCE:
                switch (Global.alliance) {
                    case RED:
                        underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                        break;
                    case BLUE:
                        underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                        break;
                }
                break;
            case GREEN: // undefined
                underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case YELLOW: // semi-autonomous unavailable
                underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            case WHITE: // autonomous active
                underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                break;
            case RAINBOW: // used for debugging maybe?
                underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
                break;
        }
    }

    public enum Color {
        ALLIANCE,
        YELLOW,
        GREEN,
        WHITE,
        RAINBOW
    }
}
