package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.navigation.Course;
import org.firstinspires.ftc.teamcode.util.Logging;

@Autonomous(name = "Drive Base Test")
public class DriveBaseTest extends LinearOpMode {
  public MecanumDriveBase driveBase;

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        RobotLog.i("Initializing Drive Base and OpMode");
        driveBase = new MecanumDriveBase(hardwareMap);
        runtime.reset();
        telemetry.setAutoClear(false);

        RobotLog.i("Waiting for Start...");
        waitForStart();
        RobotLog.i("Running opBeforeLoop");
        Logging.logMotorInfo("Left Front", driveBase.motorLeftFront.getCurrentPosition(), driveBase.motorLeftFront.getDirection(), 0, 0, telemetry);

        if (!isStopRequested() && opModeIsActive()) {
            RobotLog.i("Running opLoop");
            if (!isStopRequested() && opModeIsActive()) {
                try {
                    RobotLog.i("Trying to run a square.");

                    int howManyInches = 12;
                    double howManySeconds = 2.0;
                    square(howManySeconds, DriveBase.DriveSpeed.SLOW);
                    //simpleRun(howManyInches, driveBase.getDriveSpeedPower(DriveBase.DriveSpeed.FAST));

                    //idle(3);

                    //square(howManyInches, DriveBase.DriveSpeed.SLOW);

                } catch (Exception e) {
                    telemetry.addData("EXCEPTION: ", e);
                    RobotLog.e(e.getMessage());
                    RobotLog.logStackTrace(e);
                }
            }

        }

        RobotLog.i("Running opAfterLoop");
        driveBase.stop();
        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Elapsed Time", runtime.seconds());
        telemetry.update();
    }

    private void simpleRun(int inchesDistance, double power) throws InterruptedException {
        RobotLog.i("Simple run");

        Course course = new Course();
        Course.Leg legForward = new Course.Leg();
        legForward.name = "Forward";
        legForward.direction = DriveBase.TravelDirection.forward;
        legForward.seconds = 3;
        legForward.power = power;
        course.add(legForward);

        course.run(driveBase, runtime, this);

        RobotLog.i("finished simple run");
    }

    private void square(double howManySeconds, DriveBase.DriveSpeed howFast) {
        RobotLog.i("Running four sides by time");
        double power = driveBase.getDriveSpeedPower(howFast);

        double startRun = runtime.seconds();
        RobotLog.i("Starting forward");
        telemetry.addLine("Driving Forward");
        telemetry.update();

        // define the whole course.
        Course course = new Course();
        Course.Leg legForward = new Course.Leg();
        legForward.name = "Forward";
        legForward.direction = DriveBase.TravelDirection.forward;
        legForward.seconds = howManySeconds;
        legForward.power = power;
        course.add(legForward);

        Course.Leg legRight = new Course.Leg();
        legRight.name = "Strafe Right";
        legRight.direction = DriveBase.TravelDirection.strafeRight;
        legRight.seconds = howManySeconds;
        legRight.power = power;
        course.add(legRight);

        Course.Leg legReverse = new Course.Leg();
        legReverse.name = "Reverse";
        legReverse.direction = DriveBase.TravelDirection.reverse;
        legReverse.seconds = howManySeconds;
        legReverse.power = power;
        course.add(legReverse);

        Course.Leg legLeft = new Course.Leg();
        legLeft.name = "Strafe Left";
        legLeft.direction = DriveBase.TravelDirection.forward;
        legLeft.seconds = howManySeconds;
        legLeft.power = power;
        course.add(legLeft);

        // run our full course
        course.run(driveBase, runtime, this);

        // the course will stop anyway, whenever it finishes a leg.
        driveBase.stop();
        startRun = runtime.seconds();
        RobotLog.i("Stopping");
        telemetry.addLine("Stopping");
        telemetry.update();

    }

    private double idle(double seconds){
        double endRun = runtime.seconds();
        double startRun = runtime.seconds();

        while (endRun - startRun < 3) {
            idle();
            endRun = runtime.seconds();
        }
        return endRun - startRun;
    }


    private Telemetry.Line logMotorInfo(String name, int position, DcMotorSimple.Direction direction, int encoderTicksToRun, double power) {
        return Logging.logMotorInfo(name, position, direction, encoderTicksToRun, power, telemetry);
    }

    private void square(int howManyInches, DriveBase.DriveSpeed howFast) {

        RobotLog.i("Running four sides by distance.");
        // starting information;
        double startRun = runtime.seconds();
        int[] currentPosition;
        DcMotorSimple.Direction[] currentConfig;

        // needed to set the course
        double dblEncoderTicks = driveBase.getEncoderValueForRobotInches(howManyInches);
        telemetry.addData("Encoder Ticks calculated as Double:", dblEncoderTicks);
        int encoderTicks = (int)dblEncoderTicks;
        telemetry.addData("Encoder Ticks calculated as Int:", encoderTicks);
        double power = driveBase.getDriveSpeedPower(howFast);

        // define the whole course.
        Course course = new Course();
        Course.Leg legForward = new Course.Leg();
        legForward.name = "Forward";
        legForward.direction = DriveBase.TravelDirection.forward;
        legForward.encoderTicks = encoderTicks;
        legForward.power = power;
        course.add(legForward);

        Course.Leg legRight = new Course.Leg();
        legRight.name = "Strafe Right";
        legRight.direction = DriveBase.TravelDirection.strafeRight;
        legRight.encoderTicks = encoderTicks;
        legRight.power = power;
        course.add(legRight);

        Course.Leg legReverse = new Course.Leg();
        legReverse.name = "Reverse";
        legReverse.direction = DriveBase.TravelDirection.reverse;
        legReverse.encoderTicks = encoderTicks;
        legReverse.power = power;
        course.add(legReverse);

        Course.Leg legLeft = new Course.Leg();
        legLeft.name = "Strafe Left";
        legLeft.direction = DriveBase.TravelDirection.forward;
        legLeft.encoderTicks = encoderTicks;
        legLeft.power = power;
        course.add(legLeft);

        // run our full course
        course.run(driveBase, runtime, this);

        // report result
        currentPosition = driveBase.getEncoderPositions();
        RobotLog.i("Current Position: " + Arrays.toString(currentPosition));
        telemetry.addLine("Current Position: " + Arrays.toString(currentPosition));

        driveBase.stop();
        RobotLog.i("Stopping");
        telemetry.addLine("Stopping");
        telemetry.update();

    }
}