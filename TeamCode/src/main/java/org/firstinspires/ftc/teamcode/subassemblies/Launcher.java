package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.util.CircularDoubleArray;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.MathEx;
import org.firstinspires.ftc.teamcode.util.Subassembly;
import org.firstinspires.ftc.teamcode.util.ToggleServo;

import java.util.LinkedList;

@Config
public class Launcher extends Subassembly {

    // hardware values that should be extracted from the CAD model
    public static double MIN_HOOD_ANGLE = 40; // degrees; also known as base_arc_angle
    public static double MAX_HOOD_ANGLE = 75; // degrees
    public static double HOOD_GEAR_RATIO = 23.0; // 23:1, 368mm:16mm

    // quadratic coefficients for power calculation function (Ax² + Bx + C)
    public static double A = 0.0;
    public static double B = 0.0;
    public static double C = 0.0;

    // PIDF coefficients for flywheel speed
    public static double kP = 0.0, kI = 0.0, kD = 0.0, kF = 0.0; // TODO: find these

    public static double ENCODER_RES = 28.0; // PPR
    public static int NUM_OF_VELOCITY_SAMPLES = 5;
    public static int VELOCITY_DIP_THRESHOLD = 150; // ticks per second
    public static double TARGET_DIFF_WARNING_THRESHOLD = 30; // RPM

    public static double HOOD_RANGE_MIN = 0.0;
    public static double HOOD_RANGE_MAX = 1.0;
    public static Servo.Direction HOOD_SERVO_DIRECTION = Servo.Direction.FORWARD;
    public static DcMotorSimple.Direction FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static double GATE_RANGE_MIN = 0.0;
    public static double GATE_RANGE_MAX = 0.5;

    private final Spindexer spindexer;
    private final Servo hoodServo;
    private final ToggleServo gateServo;
    private final DcMotorEx flywheelMotor;
    private final PIDFController flywheelPIDF = new PIDFController(kP, kI, kD, kF);
    private final CircularDoubleArray flywheelVelArray;

    private Double targetVel = 0.0;
    private double hoodAngle = MIN_HOOD_ANGLE;

    private State currentState = State.IDLE;

    private final LinkedList<Artifact> launchQueue = new LinkedList<>();

    public Launcher(OpMode opMode, Spindexer spindexer) {
        super(opMode, "Launcher");
        this.spindexer = spindexer;

        flywheelMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get("launcher");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // RUN_WITHOUT_ENCODER doesn't disable encoder readouts
        flywheelMotor.setDirection(FLYWHEEL_MOTOR_DIRECTION);

        hoodServo = opMode.hardwareMap.servo.get("hood");
        hoodServo.setDirection(HOOD_SERVO_DIRECTION);
        hoodServo.scaleRange(HOOD_RANGE_MIN, HOOD_RANGE_MAX);

        gateServo = new ToggleServo(opMode.hardwareMap.servo.get("gate"));

        gateServo.setScaleRange(GATE_RANGE_MIN, GATE_RANGE_MAX);

        flywheelVelArray = new CircularDoubleArray(NUM_OF_VELOCITY_SAMPLES);
    }

    public void update() {

        if (Global.ENABLE_TUNING_MODE) flywheelPIDF.setPIDF(kP, kI, kD, kF);

        flywheelVelArray.addValue(MathEx.toRPM(flywheelMotor.getVelocity(), ENCODER_RES));
        double flywheelVel = getVelocity();

        double flywheelPower = flywheelPIDF.calculate(flywheelVel, targetVel);
        flywheelPower = MathEx.clamp(flywheelPower, -1, 1);
        sendData("flywheel power", flywheelPower);
        flywheelMotor.setPower(flywheelPower);

        switch (currentState) {
            case IDLE: // waiting for item in queue
                if (!launchQueue.isEmpty()) {
                    switch (launchQueue.getFirst()) {
                        case GREEN:
                            if (!spindexer.alignForLaunch(Spindexer.Artifact.GREEN))
                                currentState = State.REJECTED;
                            break;
                        case PURPLE:
                            if (!spindexer.alignForLaunch(Spindexer.Artifact.PURPLE))
                                currentState = State.REJECTED;
                            break;
                        case ANY:
                            if (!spindexer.alignAnyForLaunch()) currentState = State.REJECTED;
                            break;
                    }
                    currentState = State.AWAITING_SPINDEXER;
                }
                break;
            case AWAITING_SPINDEXER: // waiting for artifact delivery from spindexer
                if (!spindexer.isBusy()) {
                    gateServo.open();
                    currentState = State.LAUNCHING;
                }
                break;
            case LAUNCHING: // waiting for artifact to launch
                if (flywheelVel - flywheelMotor.getVelocity() > VELOCITY_DIP_THRESHOLD) {
                    gateServo.close();
                    launchQueue.removeFirst();
                    if (launchQueue.isEmpty()) currentState = State.SPINDOWN;
                    else currentState = State.IDLE;
                }
                break;
            case SPINDOWN: // spinning down, all artifacts launched
                break;
            case REJECTED: // an artifact couldn't be delivered or launch was cancelled
                if (!launchQueue.isEmpty()) launchQueue.removeFirst();

                gateServo.close(); // ensure this is closed

                if (launchQueue.isEmpty()) currentState = State.SPINDOWN;
                else currentState = State.IDLE;
        }
    }

    public void control(Gamepad gamepad) {
        // launch set
        if (gamepad.rightBumperWasPressed()) {
            launchMotif();
        }
        else if (gamepad.leftBumperWasPressed()) {
            launchAll();
        }
        // single launch
        if (gamepad.aWasPressed() || gamepad.crossWasPressed()) {
            launchQueue.add(Artifact.GREEN);
        }
        if (gamepad.xWasPressed() || gamepad.squareWasPressed()) {
            launchQueue.add(Artifact.PURPLE);
        }
        if (gamepad.yWasPressed() || gamepad.triangleWasPressed()) {
            launchQueue.add(Artifact.ANY);
        }
        if (gamepad.bWasPressed() || gamepad.circleWasPressed()) {
            launchQueue.clear();
            currentState = State.REJECTED;
        }
    }

    public double autoAim(double distance) {
        // TODO
        return 0.0;
    }

    public void launchMotif() {
        if (Global.motif == null) {
            // TODO: give some warning here (with watchdog probably)
            return;
        }
        switch (Global.motif) {
            case GPP:
                launchQueue.add(Artifact.GREEN);
                launchQueue.add(Artifact.PURPLE);
                launchQueue.add(Artifact.PURPLE);
                break;
            case PGP:
                launchQueue.add(Artifact.PURPLE);
                launchQueue.add(Artifact.GREEN);
                launchQueue.add(Artifact.PURPLE);
                break;
            case PPG:
                launchQueue.add(Artifact.PURPLE);
                launchQueue.add(Artifact.PURPLE);
                launchQueue.add(Artifact.GREEN);
                break;
        }
    }

    public void launchAll() {
        int numOfArtifacts = spindexer.getNumOfArtifact(Spindexer.Artifact.GREEN) + spindexer.getNumOfArtifact(Spindexer.Artifact.PURPLE);
        for (int i = numOfArtifacts; i > 0; i--) {
            launchQueue.add(Artifact.ANY);
        }
    }

    public void launchGreen() {
        launchQueue.add(Artifact.GREEN);
    }

    public void launchPurple() {
        launchQueue.add(Artifact.PURPLE);
    }

    public void launchAny() {
        launchQueue.add(Artifact.ANY);
    }

    public void cancelLaunches() {
        launchQueue.clear();
        currentState = State.REJECTED;
    }

    /**
     * sets target RPM of the flywheel launcher
     */
    public void setTargetVelocity(double targetVel) {
        this.targetVel = targetVel;
        sendData("target flywheel velocity (RPM)", targetVel);
    }

    /**
     * gets the current velocity in RPM of the flywheel
     */
    public double getVelocity() {
        double avgVel = flywheelVelArray.getAverage();
        sendData("average flywheel velocity (RPM)", avgVel);
        return avgVel;
    }

    private void setHoodAngle(double angleInDegrees) {
        angleInDegrees = MathEx.clamp(angleInDegrees, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
        hoodAngle = angleInDegrees;
        double absoluteAngle = hoodAngle - MIN_HOOD_ANGLE;
        double servoAngle = absoluteAngle * HOOD_GEAR_RATIO;
        hoodServo.setPosition(MathEx.degreesToServoPosition(servoAngle, 1800, HOOD_RANGE_MIN, HOOD_RANGE_MAX));
    }

    public enum Artifact {
        GREEN, PURPLE, ANY
    }

    private enum State {
        IDLE, AWAITING_SPINDEXER, LAUNCHING, SPINDOWN, REJECTED
    }
}