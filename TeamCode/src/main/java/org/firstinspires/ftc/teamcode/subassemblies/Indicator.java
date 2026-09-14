package org.firstinspires.ftc.teamcode.subassemblies;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.Arrays;

@Config
public class Indicator extends Subassembly {

    public static int DEFAULT_BRIGHTNESS = 1; // 0-31

    public static boolean enabled = false;

    private static final int[] colorArray = new int[10];
    private static int[] prevColorArray = new int[10];

    private static int brightness = DEFAULT_BRIGHTNESS;
    private static int prevBrightness = -1;

    private final SparkFunLEDStick stick;


    public Indicator(OpMode opMode) {
        super(opMode,"Underglow");

        stick = hardwareMap.getAll(SparkFunLEDStick.class).get(0);
        setRobotStatus(Color.YELLOW);
        setLauncherStatus(Color.BLACK);
        setAllBrightness(DEFAULT_BRIGHTNESS);
    }

    public void update() {
        if (stick == null) return;
        if (Arrays.hashCode(prevColorArray) != Arrays.hashCode(colorArray)) {
            stick.setColors(colorArray);
            prevColorArray = colorArray.clone();
        }
        if (prevBrightness != brightness) {
            stick.setBrightness(brightness);
            prevBrightness = brightness;
        }
    }

    public void stop() {
        if (Global.alliance == null) {
            setAllColor(Color.BLACK);
        } else {
            switch (Global.alliance) {
                case RED:
                    setAllColor(Color.RED);
                    break;
                case BLUE:
                    setAllColor(Color.BLUE);
                    break;
                default:
                    setAllColor(Color.BLACK);
                    break;
            }
        }
        setAllBrightness(DEFAULT_BRIGHTNESS);
    }

    public static void setColor(int index, int color) {
        if (color != colorArray[index]) {
            colorArray[index] = color;
//            stick.setColor(index, color);
        }
    }

    public static void setAllColor(int color) {
        Arrays.fill(colorArray, color);
    }

    public static void setAllBrightness(int brightness) {
        Indicator.brightness = brightness;
    }

    public static void enable() {
        enabled = true;
        setAllBrightness(DEFAULT_BRIGHTNESS);
    }

    public static void disable() {
        enabled = false;
        setAllBrightness(0);
    }

    public static void setRobotStatus(int color) {
        setColor(3, color);
        setColor(4, color);
    }

    public static void setLauncherStatus(int color) {
        setColor(5, color);
        setColor(6, color);
    }
}
