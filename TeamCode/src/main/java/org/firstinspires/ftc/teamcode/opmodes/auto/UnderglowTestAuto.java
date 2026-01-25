package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.ColorNameLookup;
import org.firstinspires.ftc.teamcode.util.Global;

import java.util.Arrays;

@Autonomous(group = "test", preselectTeleOp = Global.PRESELECT_TELEOP)
@Config
public class UnderglowTestAuto extends LinearOpMode {

    ElapsedTime opModeTimer = new ElapsedTime();

    public void setLEDs(Underglow underglow, int[] stickArray, int[] ledArray, String[] colorName) {
        for (int stickIndex : stickArray) {
            for (int i = 0; i < ledArray.length; i++) {
                int ledIndex = ledArray[i];
                int colorIndex = colorName.length - 1; // default to last index
                if (colorName.length > i) colorIndex = i; // if within range, use the current index
                int color = ColorNameLookup.getColorByName(colorName[colorIndex]).getIntValue();
                underglow.setColor(stickIndex, ledIndex, color);
            }
        }
    }

    public class BlinkyLED implements Runnable {
        OpMode opMode;
        Underglow underglow;
        int[] stickArray, ledArray;
        ElapsedTime timer = new ElapsedTime();

        public BlinkyLED(OpMode opMode, Underglow underglow, int[] stickArray, int[] ledArray) {
            this.opMode = opMode;
            this.underglow = underglow;
            this.stickArray = stickArray;
            this.ledArray = ledArray;
            timer.reset();
        }
        public void blink() {
            Thread thread = new Thread(this);
            thread.start();
        }
        /**
         * Blinks the existing color by setting brightness to from 0 to 25 according to the timer
         */
        @Override
        public void run() {
            while (opModeIsActive()) {
                for (int stickIndex : stickArray) {
                    for (int ledIndex : ledArray) {
                        // percentage of a second, in a range of 0-25 brightness
                        int brightness = (int) Math.round(((timer.milliseconds() % 1000) / 1000) * 25);
                        underglow.setBrightness(stickIndex, ledIndex, brightness);
                    }
                }
            }
        }
    }

    public enum Period {
        AUTO,
        TELEOP,
        TELEOP_ENDGAME
    }

    public static Period currentPeriod;
    public String allianceColor = Global.Alliance.RED.name();
    public String alignedColor = "Aqua";
    public String spindexerFullColor = "Lime";
    public String atSpeedColor = "Orange";
    public String purpleArtifactColor = "Purple";
    public String greenArtifactColor = "Green";
    int[] stickArray = {0};

    public void runOpMode() {

        opModeTimer.reset();
        Global.setAlliance(Global.Alliance.BLUE);
        allianceColor = Global.getAlliance().name();

        Underglow underglow = new Underglow(this);
        idle();

        waitForStart();

        if (opModeIsActive()) {
            underglow.off();
            idle();
            underglow.setColorToAlliance();
            idle();

            while (opModeIsActive()) {
                if (this.isStopRequested()) {
                    underglow.off();
                    idle();
                }
                /*
                 * Initialize the LED stick to the alliance color.
                 * Keep the first two LEDs for the alliance.
                 */
                int[] allianceLEDs = {0, 1};
                underglow.setColor(ColorNameLookup.getColorByName(allianceColor).getIntValue());

                /*
                 * The next 2 LEDs should reflect the period:
                 *  - Autonomous: Yellow
                 *  - TeleOp - White
                 *  - TeleOp EndGame - Magenta
                 * For this test opMode, we'll run each for 10 seconds.
                 */

                int[] periodLEDs = {2, 3};
                double currentSeconds = opModeTimer.seconds();
                if ((int) currentSeconds < 10 && currentPeriod != Period.AUTO) {
                    currentPeriod = Period.AUTO;
                    setLEDs(underglow, stickArray, periodLEDs, new String[]{"Yellow"});
                } else if ((int) currentSeconds >= 10 && currentSeconds < 20 && currentPeriod != Period.TELEOP) {
                    currentPeriod = Period.TELEOP;
                    setLEDs(underglow, stickArray, periodLEDs, new String[]{"White"});
                } else if ((int) currentSeconds >=20 && currentSeconds < 30 && currentPeriod != Period.TELEOP_ENDGAME) {
                    currentPeriod = Period.TELEOP_ENDGAME;
                    setLEDs(underglow, stickArray, periodLEDs, new String[]{"Magenta"});
                }
                idle();

                telemetry.addData("Alliance", Global.getAlliance().name());
                telemetry.addData("Current Period", currentPeriod);
                telemetry.addData("Current Seconds", (int) currentSeconds);

                /*
                 * The next three are signals:
                 * - Aligned to Shoot - Aqua
                 * - Spindexer full - Lime
                 * - Flywheel at speed - Orange
                 */
                int[] alignLEDs = {4};
                if (Math.random() < 0.4) {
                    setLEDs(underglow, stickArray, alignLEDs, new String[]{alignedColor});
                    telemetry.addLine("Aligned with the Goal");
                } else {
                    setLEDs(underglow, stickArray, alignLEDs, new String[]{"BLACK"});
                }
                idle();
                int[] spinLEDs = {5};
                if (Math.random() < 0.3) {
                    setLEDs(underglow, stickArray, spinLEDs, new String[]{spindexerFullColor});
                    telemetry.addLine("Spindexer is Full");
                } else {
                    setLEDs(underglow, stickArray, spinLEDs, new String[]{"BLACK"});
                }
                idle();
                int[] flyLEDs = {6};
                if (Math.random() < 0.8) {
                    setLEDs(underglow, stickArray, flyLEDs, new String[]{atSpeedColor});
                    telemetry.addLine("Flywheel is up to speed");
                } else {
                    setLEDs(underglow, stickArray, flyLEDs, new String[]{"BLACK"});
                }
                idle();


                /*
                /*
                 * Let the last three indicate the balls that are in the spindexer, in order
                 * Green, Purple
                 */
                String[] randomArtifacts = new String[]{greenArtifactColor, purpleArtifactColor, purpleArtifactColor};
                int[] spindexerLEDs = {7, 8, 9};
                // randomize the balls every second
                String[] loadedArtifacts = new String[]{greenArtifactColor, greenArtifactColor, greenArtifactColor};
                int lastSecond = 0;
                if ((lastSecond + 2) < Math.ceil(currentSeconds)) {
                    loadedArtifacts[0] = randomArtifacts[(int) ((Math.random() * 10) % 3)];
                    loadedArtifacts[1] = randomArtifacts[(int) ((Math.random() * 10) % 3)];
                    loadedArtifacts[2] = randomArtifacts[(int) ((Math.random() * 10) % 3)];
                    lastSecond = (int) Math.ceil(currentSeconds);
                    idle();
                }
                setLEDs(underglow, stickArray, spindexerLEDs, loadedArtifacts);

                telemetry.addData("Loaded Artifacts", Arrays.toString(loadedArtifacts));
                telemetry.update();
            }

        }
    }
}

