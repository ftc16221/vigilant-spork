package org.firstinspires.ftc.teamcode.subassemblies;

import static org.firstinspires.ftc.teamcode.util.MathEx.toRPM;
import static org.firstinspires.ftc.teamcode.util.MathEx.toTicksPerSec;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.CircularDoubleArray;
import org.firstinspires.ftc.teamcode.util.Subassembly;

@Config
public class Launcher extends Subassembly {

    // quadratic coefficients for power calculation function (Ax² + Bx + C)
    public static double A = 0.0;
    public static double B = 0.0;
    public static double C = 0.0;

    // PID coefficients for flywheel speed
    public static PIDFCoefficients PIDF_COEFFICIENTS = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);

    public static Boolean ENABLE_TUNING_MODE = false;

    public static double ENCODER_RES = 28.0; // PPR
    public static int NUM_OF_VELOCITY_SAMPLES = 5;

    public static DcMotorSimple.Direction LEFT_FLYWHEEL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction RIGHT_FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static double LR_DIFF_WARNING_THRESHOLD = 50.0;
    public static double TARGET_DIFF_WARNING_THRESHOLD = 100.0;

    private final DcMotorEx leftFlywheel;
    private final DcMotorEx rightFlywheel;

    private Double targetVel = 0.0;
    private final CircularDoubleArray leftVelArray;
    private final CircularDoubleArray rightVelArray;

    public Launcher(OpMode opMode) {
        super (opMode, "Launcher");

        leftFlywheel = (DcMotorEx) opMode.hardwareMap.dcMotor.get("left_flywheel");
        rightFlywheel = (DcMotorEx) opMode.hardwareMap.dcMotor.get("right_flywheel");
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setDirection(LEFT_FLYWHEEL_DIRECTION);
        rightFlywheel.setDirection(RIGHT_FLYWHEEL_DIRECTION);

        leftVelArray = new CircularDoubleArray(NUM_OF_VELOCITY_SAMPLES);
        rightVelArray = new CircularDoubleArray(NUM_OF_VELOCITY_SAMPLES);
    }

    public void update() {

        // update flywheel velocities
        leftVelArray.addValue(toRPM(leftFlywheel.getVelocity(), ENCODER_RES));
        rightVelArray.addValue(toRPM(rightFlywheel.getVelocity(), ENCODER_RES));

        broadcastWarnings();
    }

    private void broadcastWarnings() {

        double currentLeftVel = leftVelArray.getAverage();
        double currentRightVel = rightVelArray.getAverage();

        double flywheelLRVelDiff = currentLeftVel - currentRightVel;
        if (Math.abs(flywheelLRVelDiff) > LR_DIFF_WARNING_THRESHOLD) {
            String fasterFlywheel;
            if (flywheelLRVelDiff > 0) {
                fasterFlywheel = "left";
            } else {
                fasterFlywheel = "right";
            }
            String trendDirection;
            if (fasterFlywheel.equals("left")) {
                trendDirection = "right";
            } else {
                trendDirection = "left";
            }
            double absVelDiff = Math.abs(flywheelLRVelDiff);
            telemetry.addData("Warning", "%s flywheel is %.0f RPM faster than the other. Artifacts launched may trend %s", fasterFlywheel, absVelDiff, trendDirection);
        }

        /* TODO: test this code so it can be uncommented in main branch
        double flywheelTargetVelDiff = targetVel - getAverageVelocity();
        if (Math.abs(flywheelTargetVelDiff) > TARGET_DIFF_WARNING_THRESHOLD) {
            if (targetVel != 0) {
                int diffPercent = Math.toIntExact(Math.round((getAverageVelocity() / targetVel) * 100));
                telemetry.addData("Warning", "current velocity is %.0f% of target", diffPercent);
            }
        }
        */
    }

    public void launch() {
        // TODO
    }

    /** spins up the flywheel launcher to the RPMs necessary to go the specified distance */
    public void spinUp(Double distanceCM) {
        setTargetVelocity(calculateTargetVelocity(distanceCM));
    }

    /** spins down the flywheel launcher */
    public void spinDown() {
        setTargetVelocity(0);
    }

    /** returns the necessary velocity in RPM to launch the specified distance */
    public double calculateTargetVelocity(double distanceCM) {
        double calculatedVel = A * Math.pow(distanceCM, 2) + B * distanceCM + C; // Ax² + Bx + C, when x = distance from target
        sendData("calculated flywheel velocity (RPM)", calculatedVel);
        return calculatedVel;
    }

    /** gets the current average velocity in RPM of both left and flywheels */
    public double getAverageVelocity() {
        double avgVel = (leftVelArray.getAverage() + rightVelArray.getAverage()) / 2;
        sendData("average flywheel velocity (RPM)", avgVel);
        return avgVel;
    }

    /** sets target RPM of the flywheel launcher */
    public void setTargetVelocity(double targetVel) {
        this.targetVel = targetVel;
        if (ENABLE_TUNING_MODE) {
            leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF_COEFFICIENTS);
            rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF_COEFFICIENTS);
        }
        leftFlywheel.setVelocity(toTicksPerSec(targetVel, ENCODER_RES));
        rightFlywheel.setVelocity(toTicksPerSec(targetVel, ENCODER_RES));
        sendData("target flywheel velocity (RPM)", targetVel);
    }
}