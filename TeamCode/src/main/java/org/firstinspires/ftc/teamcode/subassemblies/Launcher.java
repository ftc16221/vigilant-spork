package org.firstinspires.ftc.teamcode.subassemblies;

import static org.firstinspires.ftc.teamcode.util.MathKt.toRPM;
import static org.firstinspires.ftc.teamcode.util.MathKt.toTicksPerSec;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.Subassembly;

@Config
@Configurable
public class Launcher extends Subassembly {

    // quadratic coefficients for power calculation function (Ax² + Bx + C)
    public static double A = 0.0;
    public static double B = 0.0;
    public static double C = 0.0;

    // PID coefficients for flywheel speed
    public static PIDFCoefficients PIDF_COEFFICIENTS = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);

    public static Boolean ENABLE_TUNING_MODE = false;

    public static double ENCODER_RES = 28.0; // PPR

    private final DcMotorEx flywheelMotor;

    public Launcher(OpMode opMode) {
        super (opMode, "Launcher");
        flywheelMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get("launcher");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void launch() {
        // TODO
    }

    /** Spins up the flywheel launcher to the RPMs necessary to go the specified distance */
    public void spinUp(Double distanceCM) {
        setTargetRPM(calculateTargetRPM(distanceCM));
    }

    /** Spins down the flywheel launcher */
    public void spinDown() {
        setTargetRPM(0);
    }

    /** returns the necessary RPM to launch the specified distance */
    public double calculateTargetRPM(double distanceCM) {
        double calculatedRPM = A * Math.pow(distanceCM, 2) + B * distanceCM + C; // Ax² + Bx + C, when x = distance from target
        sendData("calculated flywheel RPM", calculatedRPM);
        return calculatedRPM;
    }

    /** gets current RPM of flywheel launcher */
    public double getCurrentRPM() {
        double currentRPM = toRPM(flywheelMotor.getVelocity(), ENCODER_RES);
        sendData("current flywheel RPM", currentRPM);
        return currentRPM;
    }

    /** sets target RPM of the flywheel launcher */
    public void setTargetRPM(double targetRPM) {
        if (ENABLE_TUNING_MODE) {
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF_COEFFICIENTS);
        }
        flywheelMotor.setVelocity(toTicksPerSec(targetRPM, ENCODER_RES));
        sendData("target flywheel RPM", targetRPM);
    }
}