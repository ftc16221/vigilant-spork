package org.firstinspires.ftc.teamcode.subassemblies;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Subassembly;

//public class Spindexer extends Subassembly {
//
//    public static double ENCODER_RES = 384.5;
//
//    public static float PURPLE_LOWER_BOUND =
//
//    private DcMotor spindexerMotor;
//    private ColorSensor colorSensor;
//
//    public Spindexer(OpMode opMode) {
//        super(opMode, "Spindexer");
//
//        spindexerMotor = opMode.hardwareMap.dcMotor.get("spindexer");
//        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color_sensor");
//
//    }
//
//    private float[] getHSV() {
//        float[] hsv = new float[3];
//        Color.colorToHSV(colorSensor.argb(), hsv);
//        return hsv;
//    }
//
//
//}
