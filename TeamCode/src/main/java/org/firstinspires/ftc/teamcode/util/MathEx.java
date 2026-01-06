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

    /**
     * Find the servo position to move a specified number of degrees
     *
     * @param degrees           degrees to move servo
     * @param maxRangeInDegrees max range of unscaled servo (ie. 300 for normal goBILDA servo, 1800 for 5-turn)
     * @param scaleRangeMin     minimum value of {@code yourServo.scaleRange}
     * @param scaleRangeMax     maximum value of {@code yourServo.scaleRange}
     */
    public static double degreesToServoPosition(double degrees, double maxRangeInDegrees, double scaleRangeMin, double scaleRangeMax) {
        double scale = Math.abs(scaleRangeMax - scaleRangeMin);
        double rawServoPos = degrees / maxRangeInDegrees;
        double scaledServoPos = rawServoPos / scale;
        return clamp(scaledServoPos, 0.0, 1.0);
    }

    /**
     * normalize an angle assuming it's unit is the same as Global.ANGLE_UNIT. If not, use angleUnit.normalize(angle)
     */
    public static double normalize(double angle) {
        return Global.ANGLE_UNIT.normalize(angle);
    }

    public static double powerCurve(double value) {
        return Math.copySign(Math.pow(value, 2), value);
    }

    /**
     * from encoder ticks per second to revolutions per minute
     */
    public static double toRPM(double ticksPerSec, double encoderResolution) {
        return ticksPerSec * 60 / encoderResolution; // convert to ticks/min, then to revs/min
    }

    public static double toEncoderTicksPerSec(double rpm, double encoderResolution) {
        return rpm / 60 * encoderResolution; // convert to revs/sec, then ticks/sec
    }

    /**
     * from revolutions per minute to encoder ticks per second
     */
    public static double toTicksPerSec(double rpm, double encoderResolution) {
        return rpm / 60 * encoderResolution; // convert to revs/sec, then to ticks/sec
    }
}