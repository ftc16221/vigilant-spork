package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.CircularDoubleArray;
import org.firstinspires.ftc.teamcode.util.Subassembly;

@Config
public class Launcher extends Subassembly {

    // hardware values that should be extracted from the CAD model
    public static double MIN_HOOD_ANGLE = 40; // degrees; also known as base_arc_angle
    public static double MAX_HOOD_ANGLE = 75; // degrees
    public static double HOOD_GEAR_RATIO = 23.0; // 23:1, 368mm:16mm
    public static double PINION_PITCH_DIAMETER = 16.0; // mm
    public static double RACK_PITCH_DIAMETER = 368.0; // mm

    // quadratic coefficients for power calculation function (Ax² + Bx + C)
    public static double A = 0.0;
    public static double B = 0.0;
    public static double C = 0.0;

    // PIDF coefficients for flywheel speed
    public static double kP = 0.0, kI = 0.0, kD = 0.0, kF = 0.0; // TODO: find these

    public static Boolean ENABLE_TUNING_MODE = false;

    public static double ENCODER_RES = 28.0; // PPR
    public static int NUM_OF_VELOCITY_SAMPLES = 5;
    public static double TARGET_DIFF_WARNING_THRESHOLD = 30; // RPM

    public static double HOOD_RANGE_MIN = 0.0;
    public static double HOOD_RANGE_MAX = 1.0;
    public static Servo.Direction HOOD_SERVO_DIRECTION = Servo.Direction.FORWARD;
    public static DcMotorSimple.Direction FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static double GATE_RANGE_MIN = 0.0;
    public static double GATE_RANGE_MAX = 0.5;

    private final Servo hoodServo;
//    private final ToggleServo gateServo;
    private final DcMotorEx flywheelMotor;
    private final PIDFController flywheelPIDF = new PIDFController(kP, kI, kD, kF);

    private Double targetVel = 0.0;
    private final CircularDoubleArray flywheelVelArray;

    private double hoodAngle = MIN_HOOD_ANGLE;

    public Launcher(OpMode opMode) {
        super (opMode, "Launcher");

        flywheelMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get("flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // RUN_WITHOUT_ENCODER doesn't disable encoder readouts, only automatic power seeking based on encoder output
        flywheelMotor.setDirection(FLYWHEEL_MOTOR_DIRECTION);

        hoodServo = opMode.hardwareMap.servo.get("hood");
        hoodServo.setDirection(HOOD_SERVO_DIRECTION);
        hoodServo.scaleRange(HOOD_RANGE_MIN, HOOD_RANGE_MAX);
//
//        gateServo = new ToggleServo(opMode.hardwareMap.servo.get("gate"));
//
//        gateServo.setScaleRange(GATE_RANGE_MIN, GATE_RANGE_MAX);

        flywheelVelArray = new CircularDoubleArray(NUM_OF_VELOCITY_SAMPLES);
    }

    public void update() {

        if (ENABLE_TUNING_MODE) {
            flywheelPIDF.setPIDF(kP, kI, kD, kF);
        }

        // update flywheel velocities
//        flywheelVelArray.addValue(toRPM(flywheelMotor.getVelocity(), ENCODER_RES));
        double flywheelVel = getVelocity();
        double flywheelPower = flywheelPIDF.calculate(flywheelVel, targetVel);
        sendData("flywheel power", flywheelPower);
        flywheelMotor.setPower(flywheelPower);

        broadcastWarnings();
    }

    // TODO: test this code so it can be uncommented in main branch
    private void broadcastWarnings() {

//        double flywheelTargetVelDiff = targetVel - getAverageVelocity();
//        if (Math.abs(flywheelTargetVelDiff) > TARGET_DIFF_WARNING_THRESHOLD) {
//            if (targetVel != 0) {
//                int diffPercent = Math.toIntExact(Math.round((getAverageVelocity() / targetVel) * 100));
//                telemetry.addData("Warning", "current velocity is %.0f% of target", diffPercent);
//            }
//        }

    }

    public double autoAim(double distance) {
        // TODO
        return 0.0;
    }

    /** sets target RPM of the flywheel launcher */
    public void setTargetVelocity(double targetVel) {
        this.targetVel = targetVel;
        sendData("target flywheel velocity (RPM)", targetVel);
    }

    /** gets the current velocity in RPM of the flywheel */
    public double getVelocity() {
        double avgVel = flywheelVelArray.getAverage();
        sendData("average flywheel velocity (RPM)", avgVel);
        return avgVel;
    }
//
//    public void setHoodAngle(double angleInDegrees) {
//        angleInDegrees = MathEx.clamp(angleInDegrees, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
//        hoodAngle = angleInDegrees;
//        double absoluteAngle = hoodAngle - MIN_HOOD_ANGLE;
//        double servoAngle = absoluteAngle * HOOD_GEAR_RATIO;
//        hoodServo.setPosition(degreesToServoPosition(servoAngle, 1800, HOOD_RANGE_MIN, HOOD_RANGE_MAX));
//    }
//
//    public double getHoodAngle() { return hoodAngle; }
//
//    public void openGate() { gateServo.open(); }
//    public void closeGate() { gateServo.close(); }
//    public void toggleGate() { gateServo.toggle(); }
}