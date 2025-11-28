package org.firstinspires.ftc.teamcode.util;

public class Conversions {
    public static double mmToInches(double mm) {
        return mm / mmPerInch;
    }
    public static double inchesToMm(double inches) {
        return inches * mmPerInch;
    }
    public static final float mmPerInch        = 25.4f;
}