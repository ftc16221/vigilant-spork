package org.firstinspires.ftc.teamcode.subassemblies;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Underglow extends Subassembly {

    /**
     * Represents a multi-LED stick
     *
     */
    public class LEDStick {

        /**
         * Constructor for SparkFunLEDStick. Sets values and turns it on.
         * @param ledStick a SparkFunLEDStick instance, from the Hardware Map
         * @param numLeds number of LEDs on the stick (probably 10, for a 10-led stick)
         * @param color the color to initialize all of them to
         * @param brightness the brightness to initialize all of them to
         */
        public LEDStick(SparkFunLEDStick ledStick, int numLeds, int color, int brightness) {
            this.ledStick = ledStick;

            LightEmittingDiode[] initLEDs = new LightEmittingDiode[numLeds];
            Arrays.fill(initLEDs, new LightEmittingDiode(color, brightness));

            lightEmittingDiodes = Arrays.asList(initLEDs);

            ledStick.setColor(color);
            ledStick.setBrightness(brightness);
        }

        /**
         * data structure for storing the values to apply to each LED in a multi-led stick.
         */
        public class LightEmittingDiode {
            public int color = 0;
            public int brightness = 0;
            public LightEmittingDiode(int color, int brightness) {
                this.color = color;
                this.brightness = brightness;
            }
        }

        SparkFunLEDStick ledStick;
        List<LightEmittingDiode> lightEmittingDiodes;

        /**
         * Reset all LEDs to default color and brightness
         */
        public void setAllLedsTo(int color, int brightness) {
            setColor(color);
            setBrightness(brightness);
        }

        /**
         * setLed stores the color and brightness value for the indexed LED.
         * @param ledIndex the LED in the multi-led stick
         * @param color the color value, i.e. Color.WHITE
         * @param brightness the brightness value, 0-30
         */
        public void setLed(int ledIndex, int color, int brightness) {
            LightEmittingDiode lightEmittingDiode = getLed(ledIndex);
            if (lightEmittingDiode == null) {
                // if it should exist, create it.
                if (ledIndex > 0 && ledIndex < lightEmittingDiodes.size())
                    lightEmittingDiode = new LightEmittingDiode(color, brightness);
                else {
                    RobotLog.e("Tried to set values for non-existent LED ", ledIndex);
                    return;
                }
            }
            lightEmittingDiode.color = color;
            lightEmittingDiode.brightness = brightness;
            lightEmittingDiodes.set(ledIndex, lightEmittingDiode);
            ledStick.setColor(ledIndex, color);
            ledStick.setBrightness(ledIndex, brightness);

        }

        /**
         * getLed returns the specified LED from the list
         * @param ledIndex the index of the desired LED
         * @return LightEmittingDiode data structure
         */
        public LightEmittingDiode getLed(int ledIndex) {
            try {
                return lightEmittingDiodes.get(ledIndex);
            } catch (Exception e) {
                RobotLog.e(e.getMessage());
            }
            return null;
        }

        /**
         * Sets the physical LED on the stick to its stored values.
         * @param ledIndex the desired LED to activate
         */
        public void activateLed(int ledIndex) {
            if (ledIndex == -1) {
                setAllLedsTo(defaultColor, defaultBrightness);
            }
            LightEmittingDiode led = getLed(ledIndex);
            if (led != null) {
                ledStick.setColor(ledIndex, led.color);
                ledStick.setBrightness(ledIndex, led.brightness);
            }
        }

        /**
         * applies the stored values to all LEDs in the stick.
         */
        public void activateLeds() {
            for (int i = 0; i < lightEmittingDiodes.size(); i++) {
                activateLed(i);
            }
        }

        /**
         * setLeds stores both color and brightness values for all LEDs,
         * and applies the values to the stick itself.
         * @param color
         * @param brightness
         */
        public void setLeds(int color, int brightness) {
            for (int i = 0; i < lightEmittingDiodes.size(); i++) {
                setLed(i, color, brightness);
            }
            ledStick.setColor(color);
            ledStick.setBrightness(brightness);
        }

        /**
         * setColor stores the color of all leds in the stick, and applies the color to the stick itself.
         * @param color the color value to apply, i.e. Color.WHITE
         */
        public void setColor(int color) {
            for (int i = 0; i < lightEmittingDiodes.size(); i++) {
                setColor(color, i);
            }
            ledStick.setColor(color);
        }

        /**
         * setColor stores the color value of the specific led and applies it to the led on the stick
         * @param ledIndex the index of the specific LED in the multi-led stick
         * @param color color value, i.e. Color.WHITE
         */
        private void setColor(int ledIndex, int color) {
            LightEmittingDiode led = getLed(ledIndex);
            if (led != null) {
                led.color = color;
                ledStick.setColor(ledIndex, color);
            }
        }

        /**
         * setBrightness stores the brightness value of all leds in the stick,
         * and applies the brightness value to the stick itself.
         * @param brightness the brightness value, 0-30
         */
        public void setBrightness(int brightness) {
            for (int i = 0; i < lightEmittingDiodes.size()-1; i++) {
                setBrightness(brightness, i);
            }
            ledStick.setBrightness(brightness);
        }

        /**
         * setBrightness stores the brightness value of the specific led.
         * @param ledIndex the index of the specific LED in the multi-led stick
         * @param brightness the brightness value, 0-30
         */
        private void setBrightness(int ledIndex, int brightness) {
            LightEmittingDiode led = getLed(ledIndex);
            if (led != null) {
                led.brightness = brightness;
                ledStick.setBrightness(ledIndex, brightness);
            }
        }
    }

    public static boolean enabled = false;

    List<LEDStick> ledSticks;

    private int lastColor;

    private static final int defaultColor = Color.BLACK;
    private static final int defaultBrightness = 1;
    private static final int numLeds = 10;

    public Underglow(OpMode opMode) {
        super(opMode,"Underglow");

        List<SparkFunLEDStick> mappedSticks = hardwareMap.getAll(SparkFunLEDStick.class);
        ledSticks = new ArrayList<>(mappedSticks.size());
        for (SparkFunLEDStick stick : mappedSticks) {
            LEDStick ledStick = new LEDStick(stick, numLeds, getAllianceColor(), defaultBrightness);
            ledSticks.add(ledStick);
        }

    };

    public int getAllianceColor() {
        if (Global.alliance == Global.Alliance.BLUE) {
            return Color.BLUE;
        } else if (Global.alliance == Global.Alliance.RED) {
            return Color.RED;
        }
        return Color.BLACK;
    };

    /**
     * sets the color of all LED sticks
     * @param color the desired color in hexadecimal (ie. green = 0xFF00FF00), for alliance use -1
     */
    public void setColor(int color) {
        if (color == lastColor) return; // only set strip color if it has changed
        else if (color == -1) {
            color = getAllianceColor();
        }
        for (LEDStick ledStick : ledSticks) {
            ledStick.setColor(color);
        }
        lastColor = color;
    }

    public void setColor(int stickIndex, int ledIndex, int color) {
        if (color == -1) color = getAllianceColor();
        ledSticks.get(stickIndex).setColor(ledIndex, color);
    }

    public void setBrightness(int brightness) {
        for (LEDStick ledStick: ledSticks) {
            ledStick.setBrightness(brightness);
        }
    }
    public void setBrightness(int stickIndex, int ledIndex, int brightness) {
        ledSticks.get(stickIndex).setBrightness(ledIndex, brightness);
    }

    public void setColorToAlliance() {
        setColor(getAllianceColor());
    }

    public void disable() {
        enabled = false;
        for (LEDStick ledStick : ledSticks) {
            ledStick.ledStick.turnAllOff();
        }
    }
    public boolean isEnabled() {
        return enabled;
    }

    public void enable(int brightness) {
        enabled = true;
        for (LEDStick ledStick : ledSticks) {
            ledStick.activateLeds();
        }
    }

}
