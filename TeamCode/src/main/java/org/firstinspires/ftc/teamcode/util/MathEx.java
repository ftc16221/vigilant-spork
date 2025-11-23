package org.firstinspires.ftc.teamcode.util;

public class MathEx {

    public static double clamp(double value, double lowerBound, double upperBound) {
        return Math.max(lowerBound, Math.min(upperBound, value));
    }

    public static double encoderPositionToDegrees(int encoderPosition, double encoderResolution) {
        return 360 * encoderPosition / encoderResolution;
    }

    public static int degreesToEncoderPosition(double degrees, double encoderResolution) {
        return Math.toIntExact(Math.round(degrees / 360 * encoderResolution));
    }

    /** assumes use of a 300 degree non-continuous servo */
    public static double degreesToServoPosition(double degrees, double scaleRangeMin, double scaleRangeMax) {
        double scale = Math.abs(scaleRangeMax - scaleRangeMin);
        return degrees / (scale * 300) - (0.5 * scale);
    }

    /** assumes use of a 300 degree non-continuous servo, with a scale range of 0.0 to 1.0 */
    public static double degreesToServoPosition(double degrees) {
        return degreesToServoPosition(degrees, 0.0, 1.0);
    }

    /** normalize an angle assuming it's unit is the same as Global.ANGLE_UNIT. If not, use angleUnit.normalize(angle) */
    public static double normalize(double angle) {
        return Global.ANGLE_UNIT.normalize(angle);
    }

    public static double powerCurve(double value) {
        return Math.copySign(Math.pow(value, 2), value);
    }

    /** from encoder ticks per second to revolutions per minute */
    public static double toRPM(double ticksPerSec, double encoderResolution) {
        return ticksPerSec * 60 / encoderResolution; // convert to ticks/min, then to revs/min
    }

    /** from revolutions per minute to encoder ticks per second */
    public static double toTicksPerSec(double rpm, double encoderResolution) {
        return rpm / 60 * encoderResolution; // convert to revs/sec, then to ticks/sec
    }
}