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

    private boolean isActive = true;

    private static final int[] colorArray = new int[10];
    private static final int[] brightnessArray = new int[10];

    private static SparkFunLEDStick stick;


    public Indicator(OpMode opMode) {
        super(opMode,"Underglow");

        stick = hardwareMap.getAll(SparkFunLEDStick.class).get(0);
        updateMotifAndAlliance();
        setRobotStatus(Color.YELLOW);
        setLauncherStatus(Color.BLACK);
        setAllBrightness(DEFAULT_BRIGHTNESS);
    }

    public void stop() {
        isActive = false;
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
        if (stick == null) return;
        if (color != colorArray[index]) {
            colorArray[index] = color;
            stick.setColor(index, color);
        }
    }

    public static void setBrightness(int index, int brightness) {
        if (!enabled || stick == null) return;
        if (brightness != brightnessArray[index]) {
            brightnessArray[index] = brightness;
            stick.setBrightness(index, brightness);
        }
    }

    public static void setAllColor(int color) {
        if (stick == null) return;
        Arrays.fill(colorArray, color);
        stick.setColor(color);
    }

    public static void setAllBrightness(int brightness) {
        if (stick == null) return;
        Arrays.fill(brightnessArray, brightness);
        stick.setBrightness(brightness);
    }

    public static void enable() {
        enabled = true;
        setAllBrightness(DEFAULT_BRIGHTNESS);
    }

    public static void disable() {
        enabled = false;
        setAllBrightness(0);
    }

    public static void setIndexedArtifacts(Spindexer.Artifact[] artifacts) {
        if (artifacts.length != 3) {
            Watchdog.w("Artifact array is invalid length");
            return;
        }
        for (int i = 0; i < artifacts.length; i++) {
            int color;
            switch (artifacts[i]) {
                case PURPLE:
                    color = 0xFF800080; // PURPLE
                    break;
                case GREEN:
                    color = Color.GREEN;
                    break;
                default:
                    color = Color.BLACK;
                    break;
            }
            setColor(i + 7, color);
        }
    }

    public static void setIndexedArtifacts(Spindexer.Artifact[] artifacts, int activeArtifact) {
        setIndexedArtifacts(artifacts);
        if (activeArtifact > 2 || activeArtifact < 0) {
            Watchdog.w("Invalid activeArtifact param");
            return;
        }
        for (int i = 0; i < artifacts.length; i++) {
            if (i == activeArtifact) {
                setBrightness(i, 31);
            } else {
                setBrightness(i, DEFAULT_BRIGHTNESS);
            }
        }
    }

    public static void setRobotStatus(int color) {
        setColor(3, color);
        setColor(4, color);
    }

    public static void setLauncherStatus(int color) {
        setColor(5, color);
        setColor(6, color);
    }

    public static void updateMotifAndAlliance() {
        int allianceColor;
        if (Global.alliance == null) {
            allianceColor = Color.BLACK;
        } else {
            switch (Global.alliance) {
                case RED:
                    allianceColor = Color.RED;
                    break;
                case BLUE:
                    allianceColor = Color.BLUE;
                    break;
                default:
                    allianceColor = Color.BLACK;
                    break;
            }
        }
        switch (Global.motif) {
            case GPP:
                setColor(0, Color.BLACK);
                setColor(1, allianceColor);
                setColor(2, allianceColor);
                break;
            case PGP:
                setColor(0, allianceColor);
                setColor(1, Color.BLACK);
                setColor(2, allianceColor);
                break;
            case PPG:
                setColor(0, allianceColor);
                setColor(1, allianceColor);
                setColor(2, Color.BLACK);
                break;
            default:
                setColor(0, allianceColor);
                setColor(1, allianceColor);
                setColor(2, allianceColor);
                break;
        }
    }
}
