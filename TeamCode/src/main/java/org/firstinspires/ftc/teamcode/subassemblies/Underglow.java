package org.firstinspires.ftc.teamcode.subassemblies;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.List;

@Config
public class Underglow extends Subassembly {

    public static int BRIGHTNESS = 25; // 0-31

    public static boolean enabled = false;

    List<SparkFunLEDStick> ledSticks;
    private int lastColor;
    private int lastBrightness = BRIGHTNESS;

    public Underglow(OpMode opMode) {
        super(opMode,"Underglow");

        ledSticks = hardwareMap.getAll(SparkFunLEDStick.class);

        setBrightness(BRIGHTNESS);
        setColorToAlliance();
    }

    /**
     * sets the color of all LED sticks
     * @param color the desired color in hexadecimal (ie. green = 0xFF00FF00), for alliance use -1
     */
    public void setColor(int color) {
        if (color == lastColor) return; // only set strip color if it has changed
        else if (color == -1) {
            setColorToAlliance();
        } else {
            for (SparkFunLEDStick ledStick : ledSticks) {
                ledStick.setColor(color);
            }
        }
        lastColor = color;
    }

    public void setBrightness(int brightness) {
        if (brightness == lastBrightness) return;
        else {
            for (SparkFunLEDStick ledStick : ledSticks) {
                ledStick.setBrightness(brightness);
            }
        }
        lastBrightness = brightness;
    }

    public void setColorToAlliance() {
        if (Global.alliance == Global.Alliance.BLUE) {
            setColor(Color.BLUE);
        } else if (Global.alliance == Global.Alliance.RED) {
            setColor(Color.RED);
        }
    }

    public void disable() {
        enabled = false;
        for (SparkFunLEDStick ledStick : ledSticks) {
            ledStick.turnAllOff();
        }
    }

    public void enable() {
        enabled = true;
        for (SparkFunLEDStick ledStick : ledSticks) {
            ledStick.setColor(lastColor);
        }
    }
}
