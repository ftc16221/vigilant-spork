package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;




/**
 * Some logging utility methods for use throughout the project.
 */
public class Logging {

    /**
     * Log motor position, direction, speed, etc. to Telemetry and the standard Log.
     * @param name
     * @param position
     * @param direction
     * @param encoderTicksToRun
     * @param power
     * @param telemetry
     * @return
     */
    public static Telemetry.Line logMotorInfo(String name, int position, DcMotorSimple.Direction direction, int encoderTicksToRun, double power, Telemetry telemetry) {
        String msg = (" " +
                "Name:" + name +
                "Position:" + String.valueOf(position) +
                "Direction:" + direction.toString() +
                "Ticks:" + String.valueOf(encoderTicksToRun) +
                "Power:" + String.valueOf(power));
        Telemetry.Line line = telemetry.addLine(msg);
        telemetry.update();
        RobotLog.i(msg);
        return line;
    }

    /*public static RobotLog getLog() {
        return RobotLog.;
    }
    public static void info(String message) {
        RobotLog.i(message);
    }
    public static Telemetry.Line info(String message, Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine(message);
        info(message);
        return line;
    }*/
}
