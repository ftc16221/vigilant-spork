package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.util.Logging;

import java.util.ArrayList;

public class Course extends ArrayList<Course.Leg> {
    /**
     * Use in arrays for predefined course layout.
     */
    public static class Leg {
        public String name;
        public DriveBase.TravelDirection direction;
        public int encoderTicks;
        public double seconds;
        public double power;
    }

    /**
     * Run this course on the given drivebase, using the provided opMode and timer.
     * @param driveBase
     * @param runtime This should be defined in your opMode.
     * @param opMode Linear OpMode or child.
     */
    public void run(DriveBase driveBase, ElapsedTime runtime, LinearOpMode opMode) {

        Telemetry telemetry = opMode.telemetry;

        int[] currentPosition;
        DcMotorSimple.Direction[] currentConfig;

        // start to run the course.
        for (int c = 0; c < this.size(); c++) {
            if (!opMode.opModeIsActive() || opMode.isStopRequested())
            {
                break;
            }
            Leg leg = this.get(c);
            double startRun = runtime.seconds();

            RobotLog.i("Starting Course Leg " + leg.name);
            telemetry.addData("Running Course Leg", leg.name);

            // information about our position and configuration.
            currentPosition = driveBase.getEncoderPositions();
            currentConfig = driveBase.getMotorConfigurations(leg.direction);

            for (int i = 0; i < currentPosition.length; i++) {
                Logging.logMotorInfo("Motor " + String.valueOf(i), currentPosition[i], currentConfig[i], leg.encoderTicks, leg.power, telemetry);
            }

            // report
            Telemetry.Line[] tLines = new Telemetry.Line[currentPosition.length];
            for (int i = 0; i < currentPosition.length; i++) {
                tLines[i] = Logging.logMotorInfo("Motor " + String.valueOf(i), currentPosition[i], currentConfig[i], leg.encoderTicks, leg.power, telemetry);
            }

            if (leg.encoderTicks > 0) {
                // Using the Drive Base method to drive by encoders

                driveBase.go(leg.direction, leg.power, leg.encoderTicks);
                while(driveBase.isBusy() && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    opMode.idle();
                    //remove telemetry lines in tLines
                    for (Telemetry.Line tLine : tLines) {
                        telemetry.removeLine(tLine);
                    }
                    currentPosition = driveBase.getEncoderPositions();
                    for (int i = 0; i < currentPosition.length; i++) {
                        tLines[i] = Logging.logMotorInfo("Motor " + String.valueOf(i), currentPosition[i], currentConfig[i], leg.encoderTicks, leg.power, telemetry);
                    }
                }
            }
            else {
                // Drive for a length of time in a direction
                driveBase.go(leg.direction, leg.power);
                while(((runtime.seconds() - startRun) < leg.seconds) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    opMode.idle();
                    //remove telemetry lines in tLines
                    for (Telemetry.Line tLine : tLines) {
                        telemetry.removeLine(tLine);
                    }
                    currentPosition = driveBase.getEncoderPositions();
                    for (int i = 0; i < currentPosition.length; i++) {
                        tLines[i] = Logging.logMotorInfo("Motor " + String.valueOf(i), currentPosition[i], currentConfig[i], leg.encoderTicks, leg.power, telemetry);
                    }
                }
            }

            // give us a chance to do other stuff
            driveBase.stop();
            currentPosition = driveBase.getEncoderPositions();
            RobotLog.i("Finished Course Leg " + leg.name);
            telemetry.addData("Finished Course Leg", leg.name);
            telemetry.addData("Runtime:", runtime.seconds());
            telemetry.update();
            for (int i = 0; i < currentPosition.length; i++) {
                tLines[i] = Logging.logMotorInfo("Motor " + String.valueOf(i), currentPosition[i], currentConfig[i], leg.encoderTicks, leg.power, telemetry);
            }
            opMode.idle();
        }
    }


}
