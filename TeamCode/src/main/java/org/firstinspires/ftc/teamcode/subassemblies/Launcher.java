package org.firstinspires.ftc.teamcode.subassemblies;

import static org.firstinspires.ftc.teamcode.util.MathKt.toRPM;
import static org.firstinspires.ftc.teamcode.util.MathKt.toTicksPerSec;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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
    public static Boolean ENABLE_BALANCING = true;

    public static double ENCODER_RES = 28.0; // PPR
    public static int NUM_OF_VELOCITY_SAMPLES = 5;

    public static double BALANCE_THRESHOLD = 20.0; // max difference in flywheel velocity (RPM) before balancing
    public static double PLATEAU_THRESHOLD = 10.0; // max acceleration (RPM/sec (cursed unit i know)) for a flywheel to be considered spinning up

    public static DcMotorSimple.Direction LEFT_FLYWHEEL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction RIGHT_FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private final DcMotorEx leftFlywheel;
    private final DcMotorEx rightFlywheel;

    private final CircularDoubleArray leftVelArray;
    private final CircularDoubleArray rightVelArray;

    private double targetVel = 0;
    private double prevLeftVel = 0;
    private double prevRightVel = 0;
    private double leftAccel = 0;
    private double rightAccel = 0;

    private final ElapsedTime timer;

    public Launcher(OpMode opMode) {
        super (opMode, "Launcher");

        leftFlywheel = (DcMotorEx) opMode.hardwareMap.dcMotor.get("left_flywheel");
        rightFlywheel = (DcMotorEx) opMode.hardwareMap.dcMotor.get("left_flywheel");
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setDirection(LEFT_FLYWHEEL_DIRECTION);
        rightFlywheel.setDirection(RIGHT_FLYWHEEL_DIRECTION);

        leftVelArray = new CircularDoubleArray(NUM_OF_VELOCITY_SAMPLES);
        rightVelArray = new CircularDoubleArray(NUM_OF_VELOCITY_SAMPLES);

        timer = new ElapsedTime();
    }

    public void update() {
        double dt = timer.seconds(); // change in time
        timer.reset();

        // update flywheel velocities
        leftVelArray.addValue(toRPM(leftFlywheel.getVelocity(), ENCODER_RES));
        rightVelArray.addValue(toRPM(rightFlywheel.getVelocity(), ENCODER_RES));

        double currentLeftVel = getLeftVelocity();
        double currentRightVel = getRightVelocity();

        if (dt > 0) { // avoid unlikely divide by zero
            leftAccel = Math.abs((currentLeftVel - prevLeftVel) / dt);
            rightAccel = Math.abs((currentRightVel - prevRightVel) / dt);
        }

        prevLeftVel = currentLeftVel;
        prevRightVel = currentRightVel;

        if (targetVel != 0 && ENABLE_BALANCING) {
            balanceVel();
        }
    }

    public void launch() {
        // TODO
    }

    /** balances flywheel speeds so if one cannot reach max velocity it will bring the other's down to match the slower velocity */
    private void balanceVel() {
        double leftVel = Math.abs(getLeftVelocity());
        double rightVel = Math.abs(getRightVelocity());
        double difference = Math.abs(leftVel - rightVel);
        if (difference < BALANCE_THRESHOLD) {
            return; // not necessary to correct
        }

        boolean leftPlateaued = getLeftAcceleration() < PLATEAU_THRESHOLD;
        boolean rightPlateaued = getRightAcceleration() < PLATEAU_THRESHOLD;

        if (!leftPlateaued || !rightPlateaued) { // left or right have not plateaued in acceleration don't bother balancing
            return;
        }

        double slowerVel = Math.min(leftVel, rightVel);
        setTargetVelocity(slowerVel);
        RobotLog.w("(Launcher) Substantial difference in flywheel velocity detected (L: %.0f, R: %.0f). Setting both flywheels to %.0f", leftVel, rightVel, slowerVel);
    }

    /** Spins up the flywheel launcher to the RPMs necessary to go the specified distance */
    public void spinUp(Double distanceCM) {
        setTargetVelocity(calculateTargetVelocity(distanceCM));
    }

    /** Spins down the flywheel launcher */
    public void spinDown() {
        setTargetVelocity(0);
    }

    /** returns the necessary velocity in RPM to launch the specified distance */
    public double calculateTargetVelocity(double distanceCM) {
        double calculatedVel = A * Math.pow(distanceCM, 2) + B * distanceCM + C; // Ax² + Bx + C, when x = distance from target
        sendData("calculated flywheel velocity (RPM)", calculatedVel);
        return calculatedVel;
    }

    /** gets current velocity in RPM of right flywheel */
    public double getLeftVelocity() {
        double leftRPM = leftVelArray.getAverage();
        sendData("left flywheel velocity (RPM)", leftRPM);
        return leftRPM;
    }

    /** gets current velocity in RPM of right flywheel */
    public double getRightVelocity() {
        double rightRPM = rightVelArray.getAverage();
        sendData("right flywheel velocity (RPM)", rightRPM);
        return rightRPM;
    }

    /** gets current velocity in RPM of both flywheels (averaged) */
    public double getAverageVelocity() {
        double avgVel = (Math.abs(getLeftVelocity()) + Math.abs(getRightVelocity())) / 2;
        sendData("current flywheel RPM", avgVel);
        return avgVel;
    }

    /** gets current acceleration in RPM of left flywheel */
    public double getLeftAcceleration() {
        sendData("left flywheel acceleration (RPM/s)", leftAccel);
        return leftAccel;
    }

    /** gets current acceleration in RPM of right flywheel */
    public double getRightAcceleration() {
        sendData("right flywheel acceleration (RPM/s)", rightAccel);
        return rightAccel;
    }

    /** gets current acceleration in RPM of both flywheels */
    public double getAverageAcceleration() {
        double avgAccel = (Math.abs(getLeftAcceleration() + Math.abs(getRightAcceleration())) / 2);
        sendData("current flywheel acceleration (RPM/s)", avgAccel);
        return avgAccel;
    }

    /** sets target RPM of the flywheel launcher */
    public void setTargetVelocity(double targetVel) {
        this.targetVel = targetVel;
        if (ENABLE_TUNING_MODE) {
            leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF_COEFFICIENTS);
        }
        leftFlywheel.setVelocity(toTicksPerSec(targetVel, ENCODER_RES));
        sendData("target flywheel velocity (RPM)", targetVel);
    }
}