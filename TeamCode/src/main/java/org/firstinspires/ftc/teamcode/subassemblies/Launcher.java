package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.CircularDoubleArray;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.MathEx;
import org.firstinspires.ftc.teamcode.util.Subassembly;
import org.firstinspires.ftc.teamcode.util.ToggleServo;

import java.util.LinkedList;
import java.util.concurrent.TimeUnit;

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
    public static double kP = 0.0009, kI = 0.03, kD = 0.0, kF = 0.00021;

    public static double ENCODER_RES = 28.0; // PPR
    public static int NUM_OF_VELOCITY_SAMPLES = 5;
    public static double VELOCITY_TOLERANCE = 50; // RPM
    public static int VELOCITY_DIP_THRESHOLD = 150; // ticks per second

    public static double HOOD_RANGE_MIN = 0.0;
    public static double HOOD_RANGE_MAX = 1.0;
    public static DcMotorSimple.Direction FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static double GATE_RANGE_MIN = 0.8;
    public static double GATE_RANGE_MAX = 1.0;

    public static double KICKER_RANGE_MIN = 0.03;
    public static double KICKER_RANGE_MAX = 0.25;
    public static int KICKER_EXTENSION_TIME = 250; // milliseconds

    public static int STUCK_DETECTION_TIME = 1500; // milliseconds
    public static int SPINDEXER_MOVEMENT_DELAY = 400;

    private final Spindexer spindexer;

    private final Servo hoodServo;

    private final ToggleServo gateServo;

    private final DcMotorEx flywheelMotor;
    private final PIDFController flywheelPIDF = new PIDFController(kP, kI, kD, kF);
    private final CircularDoubleArray flywheelVelArray;

    private final ToggleServo kickerServo;
    private final Deadline kickerDeadline = new Deadline(KICKER_EXTENSION_TIME, TimeUnit.MILLISECONDS);
    private final Deadline spindexerMovementDeadline = new Deadline(SPINDEXER_MOVEMENT_DELAY, TimeUnit.MILLISECONDS);


    private final Deadline stuckDeadline = new Deadline(STUCK_DETECTION_TIME, TimeUnit.MILLISECONDS);

    private Double targetVel = 0.0;

    private State currentState = State.IDLE;

    private final LinkedList<Artifact> launchQueue = new LinkedList<>();

    public Launcher(OpMode opMode, Spindexer spindexer) {
        super(opMode, "Launcher");
        this.spindexer = spindexer;

        flywheelMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get("launcher");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // RUN_WITHOUT_ENCODER doesn't disable encoder readouts
        flywheelMotor.setDirection(FLYWHEEL_MOTOR_DIRECTION);

        hoodServo = opMode.hardwareMap.servo.get("hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);
        hoodServo.scaleRange(HOOD_RANGE_MIN, HOOD_RANGE_MAX);

        gateServo = new ToggleServo(opMode.hardwareMap.servo.get("gate"));
        gateServo.setDirection(Servo.Direction.REVERSE);
        gateServo.setScaleRange(GATE_RANGE_MIN, GATE_RANGE_MAX);

        kickerServo = new ToggleServo(opMode.hardwareMap.servo.get("kicker"));
        kickerServo.setScaleRange(KICKER_RANGE_MIN, KICKER_RANGE_MAX);

        flywheelVelArray = new CircularDoubleArray(NUM_OF_VELOCITY_SAMPLES);

        if (opMode.getClass().isAnnotationPresent(Autonomous.class)) {
            gateServo.close();
            kickerServo.close();
        }
    }

    public void update() {

        if (Global.ENABLE_TUNING_MODE) flywheelPIDF.setPIDF(kP, kI, kD, kF);

        flywheelVelArray.addValue(MathEx.toRPM(flywheelMotor.getVelocity(), ENCODER_RES));
        double flywheelVel = flywheelVelArray.getAverage();
        double power = flywheelPIDF.calculate(flywheelVel, targetVel);
        flywheelMotor.setPower(power);
        sendData("error", flywheelVel - targetVel);
        sendData("current velocity", flywheelVel);
        sendData("target velocity", targetVel);

        switch (currentState) {
            case IDLE: // waiting for item in queue
                if (!launchQueue.isEmpty() && spindexerMovementDeadline.hasExpired()) {
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
                    if (currentState == State.REJECTED) {
                        Watchdog.w("Artifact of " + launchQueue.getFirst() + " type couldn't be found");
                        break;
                    }
                    currentState = State.AWAITING_SPINDEXER;
                    Watchdog.i("Awaiting " + launchQueue.getFirst() + " artifact from spindexer");
                }
                break;
            case AWAITING_SPINDEXER: // waiting for artifact delivery from spindexer
                if (!spindexer.isBusy() && isReady()) {
                    gateServo.open();
                    kick();
                    stuckDeadline.reset();
                    currentState = State.LAUNCHING;
                    Watchdog.i("Launching artifact");
                }
                break;
            case LAUNCHING: // waiting for artifact to launch
                if (stuckDeadline.hasExpired()) {
                    kick();
                    stuckDeadline.reset();
                }
                if (flywheelVel - MathEx.toRPM(flywheelMotor.getVelocity(), ENCODER_RES) > VELOCITY_DIP_THRESHOLD) {
                    gateServo.close();
                    kickerServo.close();

                    launchQueue.removeFirst();
                    spindexer.emptyActiveSlot();
                    Watchdog.i("Artifact successfully launched");
                    if (launchQueue.isEmpty()) {
                        currentState = State.SPINDOWN;
                        Watchdog.i("All artifacts in queue launched, spinning down");
                    } else {
                        currentState = State.IDLE;
                    }
                }
                break;
            case ARTIFACT_STUCK:
                // TODO maybe shake the spindexer around or something?
                break;
            case SPINDOWN: // spinning down, all artifacts launched
                currentState = State.IDLE;
                break;
            case REJECTED:
                Watchdog.w("An artifact couldn't be delivered or launch was cancelled");
                if (!launchQueue.isEmpty()) launchQueue.removeFirst();

                gateServo.close(); // ensure this is closed
                kickerServo.close();

                if (launchQueue.isEmpty()) currentState = State.SPINDOWN;
                else currentState = State.IDLE;
        }

        if ((kickerDeadline.hasExpired() || spindexer.getMode() != Spindexer.Mode.LAUNCHER) && kickerServo.isOpen()) {
            kickerServo.close();
        }

        telemetry.addData("launcher state", currentState);
        telemetry.addData("launcher isReady", isReady());
    }

    public double autoAim(double distance) {
        // TODO
        return 0.0;
    }

    public void launchMotif() {
        if (Global.motif == Global.Motif.UNKNOWN) {
            Watchdog.e("Unable to launch motif as it is unknown");
            return;
        }
        Watchdog.i("Motif added to launch queue: " + Global.motif);
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
        Watchdog.i("All " + numOfArtifacts + " artifacts added to launch queue");
    }

    public void launchGreen() {
        launchQueue.add(Artifact.GREEN);
        Watchdog.i("Green artifact added to launch queue");
    }

    public void launchPurple() {
        launchQueue.add(Artifact.PURPLE);
        Watchdog.i("Purple artifact added to launch queue");
    }

    public void launchAny() {
        launchQueue.add(Artifact.ANY);
        Watchdog.i("Artifact of any type added to launch queue");
    }

    public void cancelLaunches() {
        launchQueue.clear();
        currentState = State.REJECTED;
        Watchdog.w("All launches cancelled");
    }

    public void kick() {
        kickerServo.open();
        spindexerMovementDeadline.reset();
        kickerDeadline.reset();
    }

    public void setTargetVelocity(double rpm) {
        targetVel = rpm;
    }

    public double getVelocity() {
        return flywheelVelArray.getAverage();
    }

    public void setHoodAngle(double angleInDegrees) {
        angleInDegrees = MathEx.clamp(angleInDegrees, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
        double absoluteAngle = angleInDegrees - MIN_HOOD_ANGLE;
        double servoAngle = absoluteAngle * HOOD_GEAR_RATIO;
        hoodServo.setPosition(MathEx.degreesToServoPosition(servoAngle, 1800, HOOD_RANGE_MIN, HOOD_RANGE_MAX));
    }

    public boolean isReady() {
        return Math.abs(flywheelVelArray.getAverage() - targetVel) < VELOCITY_TOLERANCE;
    }

    public State getState() {
        return currentState;
    }

    public ToggleServo getGateServo() {
        return gateServo;
    }

    public enum Artifact {
        GREEN, PURPLE, ANY
    }

    public enum State {
        IDLE, AWAITING_SPINDEXER, LAUNCHING, ARTIFACT_STUCK, SPINDOWN, REJECTED
    }
}