package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.MathEx;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

@Config
public class Spindexer extends Subassembly {
    // from home
    public static double INTAKE_ANGLE = 66.0; // degrees
    public static double LAUNCHER_ANGLE = 220.0; // degrees

    public static double ENCODER_RES = 537.7; // PPR

    public static float PURP_G_PCT_THRESHOLD = 37.5f; // must be less than
    public static float PURP_B_PCT_THRESHOLD = 35f; // must be greater than

    public static float GREEN_G_PCT_THRESHOLD = 45f; // must be greater than

    public static float COLOR_SENSOR_GAIN = 10f; // always >=1
    public static double PROXIMITY_THRESHOLD = 5.0; // cm

    public static double kP = 0.009, kI = 0.1, kD = 0.0007, kF = 0.0; // vibrations and instability from a high kI are actually good, to shake out stuck artifacts
    public static double TOLERANCE = 2.0; // degrees

    public static int INTAKE_SAFETY_DEADLINE = 1200; // ms
    public static int INTAKE_DEADLINE = 300; // ms

    public Artifact[] drum = new Artifact[3];

    private final Intake intake;
    private final Deadline intakeSafetyDeadline = new Deadline(INTAKE_SAFETY_DEADLINE, TimeUnit.MILLISECONDS);
    private boolean hasIntakeSafetyDeadlineExpired = false;
    private final Deadline intakeDeadline = new Deadline(INTAKE_DEADLINE, TimeUnit.MILLISECONDS);

    private final DcMotor spindexerMotor;
    private final PIDFController spindexerPIDF = new PIDFController(kP, kI, kD, kF);
    ;

    private final NormalizedColorSensor colorSensor;
    private final Artifact[] lastDetectedColors = new Artifact[10];
    private int detectedColorIndex;

    private double baseAngle = INTAKE_ANGLE;
    private int activeSlot = 0;

    private Mode mode = Spindexer.Mode.LAUNCHER;
    private Mode prevMode = mode;
    private boolean isBusy = true;

    public Spindexer(OpMode opMode, Intake intake) {
        super(opMode, "Spindexer");
        this.intake = intake;

        spindexerMotor = opMode.hardwareMap.dcMotor.get("spindexer");
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        activeSlot = 0;
        spindexerPIDF.setSetPoint(0);

        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        colorSensor.setGain(COLOR_SENSOR_GAIN);

        boolean isAutoOpMode = opMode.getClass().isAnnotationPresent(Autonomous.class);
        if (isAutoOpMode || drum[0] == null) {
            drum[0] = Artifact.GREEN;
            drum[1] = Artifact.PURPLE;
            drum[2] = Artifact.PURPLE;
        } else {
            if (isFull()) {
                mode = Spindexer.Mode.LAUNCHER;
            } else {
                mode = Spindexer.Mode.INTAKE;
            }
        }

        // TODO: temp
        drum[0] = Artifact.EMPTY;
        drum[1] = Artifact.EMPTY;
        drum[2] = Artifact.EMPTY;

        if (isFull()) mode = Spindexer.Mode.LAUNCHER;
        else mode = Spindexer.Mode.INTAKE;
    }

    public boolean isFull() {
        return getNumOfArtifact(Artifact.EMPTY) == 0;
    }

    public int getNumOfArtifact(Artifact artifact) {
        int result = 0;
        for (Artifact artifact1 : drum) {
            if (artifact1 == artifact) result++;
        }
        return result;
    }

    public void update() {

        if (Global.ENABLE_TUNING_MODE) {
            spindexerPIDF.setPIDF(kP, kI, kD, kF);
            colorSensor.setGain(COLOR_SENSOR_GAIN);
        }

        if (mode == Spindexer.Mode.INTAKE && baseAngle != INTAKE_ANGLE) baseAngle = INTAKE_ANGLE;
        else if (mode == Spindexer.Mode.LAUNCHER && baseAngle != LAUNCHER_ANGLE)
            baseAngle = LAUNCHER_ANGLE;

        double error = getDistanceFromIndex(activeSlot);

        evaluatePID(error);

        // intake mode
        Artifact detectedArtifact = getDetectedArtifact();
        if (!isFull() && mode == Spindexer.Mode.INTAKE && !isBusy && detectedArtifact != Artifact.EMPTY && intakeDeadline.hasExpired()) {
            Watchdog.i(detectedArtifact + " artifact detected in intake");
            drum[activeSlot] = detectedArtifact;
            intakeDeadline.reset();
        }

        if (mode == Spindexer.Mode.INTAKE) {
            if (isFull()) {
                Watchdog.i("Spindexer full, switching to LAUNCHER mode");
                mode = Spindexer.Mode.LAUNCHER;
                // go to first color of motif first, to save time
                int motifSlot = -1;
                if (Global.motif != null) {
                    if (Global.motif.toString().startsWith("G"))
                        motifSlot = getIndexOfClosestArtifact(Artifact.GREEN);
                    if (Global.motif.toString().startsWith("P"))
                        motifSlot = getIndexOfClosestArtifact(Artifact.PURPLE);
                }
                if (motifSlot == -1) motifSlot = getIndexOfClosestArtifact();
                activeSlot = motifSlot;
            } else {
                // get ready for another artifact
                activeSlot = getIndexOfClosestArtifact(Artifact.EMPTY);
            }
        }

        if (isEmpty() && mode == Spindexer.Mode.LAUNCHER) {
            Watchdog.i("Spindexer empty, switching to INTAKE mode");
            mode = Spindexer.Mode.INTAKE;
            activeSlot = getIndexOfClosestArtifact(Artifact.EMPTY);
        }

        if (prevMode != mode) {
            if (mode == Spindexer.Mode.INTAKE) {
                intake.setMode(Intake.Mode.IN);
            } else if (mode == Spindexer.Mode.LAUNCHER) {
                intake.setMode(Intake.Mode.OUT);
                intakeSafetyDeadline.reset();
                hasIntakeSafetyDeadlineExpired = false;
            }
            prevMode = mode;
        }

        if (mode == Spindexer.Mode.LAUNCHER && intakeSafetyDeadline.hasExpired() && !hasIntakeSafetyDeadlineExpired) {
            intake.setMode(Intake.Mode.OFF);
            hasIntakeSafetyDeadlineExpired = true;
        }
    }

    public void stop() {
        spindexerMotor.setPower(0.0);
    }

    public void runTelemetry() {
        telemetry.addData("mode", mode);
        telemetry.addData("isBusy", isBusy);
        telemetry.addLine();
        telemetry.addLine("REMINDER: slots are indexed starting with 0");
        telemetry.addData("active slot", activeSlot);
        telemetry.addData("slot 0", drum[0]);
        telemetry.addData("slot 1", drum[1]);
        telemetry.addData("slot 2", drum[2]);
        telemetry.addLine();
        telemetry.addData("detected colors", Arrays.toString(lastDetectedColors));
        double currentAngle = getCurrentAngle();
        telemetry.addData("current angle", currentAngle);
        telemetry.addData("current normalized angle", currentAngle % 360);
    }

    public void evaluatePID(double error) {
        double power = spindexerPIDF.calculate(error);
        power = MathEx.clamp(power, -1, 1);
        spindexerMotor.setPower(power);
        isBusy = error > TOLERANCE;
        sendData("spx error", error);
        sendData("spx power", power);
    }

    /**
     * get current angle of the spindexer DcMotor in degrees
     */
    public double getCurrentAngle() {
        return MathEx.encoderPositionToDegrees(spindexerMotor.getCurrentPosition(), ENCODER_RES);
    }

    /**
     * rotates the drum to align a specified artifact color with launcher
     *
     * @return whether an artifact of specified color was found
     */
    public boolean alignForLaunch(Artifact artifact) {
        mode = Spindexer.Mode.LAUNCHER;
        if (!contains(artifact)) {
            Watchdog.w("Cannot align for launch as the spindexer does not contain any " + artifact + " artifacts");
            return false;
        }
        activeSlot = getIndexOfClosestArtifact(artifact);
        return true;
    }

    /**
     * rotates the drum to align an artifact of either color with launcher
     *
     * @return whether any artifact was found
     */
    public boolean alignAnyForLaunch() {
        mode = Spindexer.Mode.LAUNCHER;
        if (isEmpty()) {
            Watchdog.w("Cannot align for launch as the spindexer is empty");
            return false;
        }
        activeSlot = getIndexOfClosestArtifact();
        return true;
    }

    /**
     * rotates the drum to align a prioritized artifact with the launcher. If there are no
     * prioritized artifacts, rotates to the other artifact color.
     *
     * @return whether any artifact was found
     */
    public boolean alignPrioritizedForLaunch(Artifact artifact) {
        mode = Spindexer.Mode.LAUNCHER;
        boolean wasSuccess = alignForLaunch(artifact);
        if (!wasSuccess) wasSuccess = alignAnyForLaunch();
        return wasSuccess;
    }

    /**
     * rotates the drum to align an empty slot with the intake
     *
     * @return whether an empty slot was found
     */
    public boolean alignForIntake() {
        mode = Spindexer.Mode.INTAKE;
        if (isFull()) {
            Watchdog.w("Cannot align to intake as the spindexer is full");
            return false;
        }
        ;
        activeSlot = getIndexOfClosestArtifact(Artifact.EMPTY);
        return true;
    }

    private int getIndexOfClosestArtifact(Artifact artifact) {
        int result = -1;
        double smallestDistance = 181;
        for (int i = 0; i < 3; i++) {
            if (drum[i] == artifact) {
                double distance = Math.abs(getDistanceFromIndex(i));
                if (distance < smallestDistance) {
                    smallestDistance = distance;
                    result = i;
                }
            }
        }
        return result;
    }

    private double getDistanceFromIndex(int slotIndex) {
        double indexAngle = baseAngle + (slotIndex * 120);
        double currentAngle = getCurrentAngle();
        double distance = (indexAngle - currentAngle) % 360;

        // normalize distance between [-179.9 to 180]
        if (distance > 180) distance -= 360;
        if (distance <= -180) distance += 360;
        return distance;
    }

    /**
     * compares the target position to current position, and if a difference of greater
     * than {@value TOLERANCE} degrees is found return true
     */
    public boolean isBusy() {
        return isBusy;
    }

    public boolean contains(Artifact artifact) {
        boolean result = false;
        for (Artifact artifact1 : drum) {
            if (artifact1 == artifact) {
                result = true;
                break;
            }
        }
        return result;
    }

    public boolean isEmpty() {
        return getNumOfArtifact(Artifact.EMPTY) == 3;
    }

    /**
     * returns the spindexer DcMotor
     */
    public DcMotor getMotor() {
        return spindexerMotor;
    }

    public NormalizedColorSensor getColorSensor() {
        return colorSensor;
    }

    public Artifact getDetectedArtifact() {

        lastDetectedColors[detectedColorIndex] = getCurrentColor();
        detectedColorIndex = (detectedColorIndex + 1) % lastDetectedColors.length;

        if (isObjectInProximity()) {
            int numOfPurpleDetections = 0;
            int numOfGreenDetections = 0;

            for (Artifact detectedColor : lastDetectedColors) {
                if (detectedColor == Artifact.PURPLE) numOfPurpleDetections++;
                else if (detectedColor == Artifact.GREEN) numOfGreenDetections++;
            }

            if (numOfPurpleDetections > 8) return Artifact.PURPLE;
            else if (numOfGreenDetections > 8) return Artifact.GREEN;
            else  /* i love alignment */        return Artifact.EMPTY;
        } else {
            return Artifact.EMPTY;
        }
    }

    public Artifact getCurrentColor() {
        NormalizedRGBA rawColors = colorSensor.getNormalizedColors();
        float normR = rawColors.red / rawColors.alpha;
        float normG = rawColors.green / rawColors.alpha;
        float normB = rawColors.blue / rawColors.alpha;
        float sum = normR + normG + normB;
        float percentG = normG / sum * 100;
        float percentB = normB / sum * 100;

        if (percentG > GREEN_G_PCT_THRESHOLD) {
            return Artifact.GREEN;
        } else if (percentG < PURP_G_PCT_THRESHOLD && percentB > PURP_B_PCT_THRESHOLD) {
            return Artifact.PURPLE;
        } else {
            return Artifact.EMPTY;
        }
    }

    public void emptyActiveSlot() {
        drum[activeSlot] = Artifact.EMPTY;
    }

    private int getIndexOfClosestArtifact() {
        int result = -1;
        double smallestDistance = 181;
        for (int i = 0; i < 3; i++) {
            if (drum[i] != Artifact.EMPTY) {
                double distance = Math.abs(getDistanceFromIndex(i));
                if (distance < smallestDistance) {
                    smallestDistance = distance;
                    result = i;
                }
            }
        }
        return result;
    }

    /**
     * checks and returns whether an object is within {@value PROXIMITY_THRESHOLD} cm of the color sensor
     */
    private boolean isObjectInProximity() {
        if (colorSensor instanceof DistanceSensor) {
            DistanceSensor distanceSensor = (DistanceSensor) colorSensor;
            return distanceSensor.getDistance(DistanceUnit.CM) <= PROXIMITY_THRESHOLD;
        } else {
            return true;
        }
    }

    public enum Artifact {
        GREEN, PURPLE, EMPTY
    }

    public enum Mode {
        INTAKE, LAUNCHER
    }
}