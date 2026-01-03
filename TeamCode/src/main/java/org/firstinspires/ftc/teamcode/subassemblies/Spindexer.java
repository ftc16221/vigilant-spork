package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.util.ColorThreshold;
import org.firstinspires.ftc.teamcode.util.Subassembly;

public class Spindexer extends Subassembly {

    public static double INTAKE_ANGLE = 0.0; // degrees
    public static double OUTTAKE_ANGLE = 180.0; // degrees TODO: find actual value

    public static double ENCODER_RES = 384.5; // PPR

    public static ColorThreshold ORANGE_THRESHOLD = new ColorThreshold(0xFF9E5C29, 0xFFFCAD03);
    public static ColorThreshold GREEN_THRESHOLD = new ColorThreshold(0xFF36572A, 0xFF03FC94);
    public static ColorThreshold PURPLE_THRESHOLD = new ColorThreshold(0xFF7855A3, 0xFFC200FF);

    private final DcMotor spindexerMotor;
    private final NormalizedColorSensor colorSensor;

    public Spindexer(OpMode opMode) {
        super(opMode, "Spindexer");

        spindexerMotor = opMode.hardwareMap.dcMotor.get("spindexer");
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
    }

    public void update() {
    }

    public enum DetectedColor {
        ORANGE, GREEN, PURPLE, UNKNOWN
    }

    public DetectedColor getDetectedColor() {
        int rawColor = colorSensor.getNormalizedColors().toColor();
        if (ORANGE_THRESHOLD.isColorInRange(rawColor)) {
            return DetectedColor.ORANGE;
        } else if (GREEN_THRESHOLD.isColorInRange(rawColor)){
            return DetectedColor.GREEN;
        } else if (PURPLE_THRESHOLD.isColorInRange(rawColor)) {
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }

    public DcMotor getMotor() { return spindexerMotor; }

    public void resetPosition() {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try {
            Thread.sleep(20);
        } catch (InterruptedException ignored) { }
        spindexerMotor.setTargetPosition(0);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}