package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ColorThreshold;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.MathEx;
import org.firstinspires.ftc.teamcode.util.Subassembly;

@Config
public class Spindexer extends Subassembly {

    public static boolean ENABLE_PASSIVE_REHOMING = false;
    public static double REHOMING_THRESHOLD = 3; // encoder pulses

    // from home
    public static double INTAKE_ANGLE = -60.0; // degrees
    public static double LAUNCHER_ANGLE = 120.0; // degrees TODO: find actual value

    public static double ENCODER_RES = 384.5; // PPR

    // color thresholds are in 0xAARRGGBB formatting
    public static ColorThreshold HOME_COLOR_THRESHOLD = new ColorThreshold(20f, 30f, 0.9f, 1.0f, 0.9f, 1.0f); // orange
    public static ColorThreshold GREEN_THRESHOLD = new ColorThreshold(130f, 150f, 0.45f, 0.7f, 0.15f, 1.0f);
    public static ColorThreshold PURPLE_THRESHOLD = new ColorThreshold(210f, 230f, 0.4f, 0.7f, 0.13f, 1.0f);

    public static float COLOR_SENSOR_GAIN = 150f; // always >=1
    public static double PROXIMITY_THRESHOLD = 5.0; // cm

    public static double kP = 0.008, kI = 0.04, kD = 0.0004, kF = 0.0; // TODO: tune with the real thing
    public static int TOLERANCE = 2; // degrees

    public static Artifact[] drum = new Artifact[3];

    private final Intake intake;

    private final DcMotor spindexerMotor;
    private final NormalizedColorSensor colorSensor;
    private final PIDFController spindexerPIDF = new PIDFController(kP, kI, kD, kF);;

    private int encoderOffset = 0; // only used when passive re-homing is enabled

    private double baseAngle = INTAKE_ANGLE;
    public static int activeSlot = 0;

    public static Mode mode = Mode.LAUNCHER; // TODO: ensure this isn't set to INTAKE
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
        if (isAutoOpMode || drum[0] == null) { // TODO: ensure this isn't all EMPTY
            drum[0] = Artifact.GREEN;
            drum[1] = Artifact.PURPLE;
            drum[2] = Artifact.PURPLE;
        } else {
            if (isFull()) {
                mode = Mode.LAUNCHER;
            } else {
                mode = Mode.INTAKE;
            }
        }

        if (isFull()) mode = Mode.LAUNCHER;
        else mode = Mode.INTAKE;
    }

    public void update() {

        if (Global.ENABLE_TUNING_MODE) {
            spindexerPIDF.setPIDF(kP, kI, kD, kF);
            colorSensor.setGain(COLOR_SENSOR_GAIN);
        }

        if (mode == Mode.INTAKE) baseAngle = INTAKE_ANGLE;
        else baseAngle = LAUNCHER_ANGLE;

        double error = getDistanceFromIndex(activeSlot);

        double power = spindexerPIDF.calculate(error);
        power = MathEx.clamp(power, -1, 1);
        sendData("spindexer power", power);
        spindexerMotor.setPower(power);
        isBusy = error > TOLERANCE;
        sendData("error", error);
        sendData("power", power);

        // intake mode
        Artifact detectedArtifact = getDetectedArtifact();
        if (!isFull() && mode == Mode.INTAKE && !isBusy && detectedArtifact != Artifact.EMPTY) {
            drum[activeSlot] = detectedArtifact;
        }

        if (mode == Mode.INTAKE) {
            if (isFull()) {
                mode = Mode.LAUNCHER;
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

        if (isEmpty() && mode == Mode.LAUNCHER) {
            mode = Mode.INTAKE;
            activeSlot = getIndexOfClosestArtifact(Artifact.EMPTY);
        }

        if (ENABLE_PASSIVE_REHOMING) {
            if (getDetectedColor() == DetectedColor.HOME_COLOR) {
                int encoderError = spindexerMotor.getCurrentPosition();
                if (encoderError >= REHOMING_THRESHOLD) {
                    encoderOffset = -encoderError;
                }
            }
        }
    }

    public void stop() {
        spindexerMotor.setPower(0.0);
    }

    public void runTelemetry() {
        telemetry.addData("mode", mode);
        telemetry.addLine();
        telemetry.addLine("REMINDER: slots are indexed starting with 0");
        telemetry.addData("active slot", activeSlot);
        telemetry.addData("slot 0", drum[0]);
        telemetry.addData("slot 1", drum[1]);
        telemetry.addData("slot 2", drum[2]);
        telemetry.addLine();
        telemetry.addData("detected color", getDetectedColor());
        telemetry.addData("is object in proximity", isObjectInProximity());
        double currentAngle = getCurrentAngle();
        telemetry.addData("current angle", currentAngle);
        telemetry.addData("current normalized angle", currentAngle % 360);
    }

    public enum DetectedColor {
        HOME_COLOR, GREEN, PURPLE, UNKNOWN
    }

    public enum Artifact {
        GREEN, PURPLE, EMPTY
    }

    public enum Mode {
        INTAKE, LAUNCHER
    }

    public void launch() {
        drum[activeSlot] = Artifact.EMPTY;
    }

    /**
     * rotates the drum to align a specified artifact color with launcher
     * @return whether an artifact of specified color was found
     */
    public boolean alignForLaunch(Artifact artifact) {
        mode = Mode.LAUNCHER;
        if (!contains(artifact)) return false;
        activeSlot = getIndexOfClosestArtifact(artifact);
        return true;
    }

    /**
     * rotates the drum to align an artifact of either color with launcher
     * @return whether any artifact was found
     */
    public boolean alignAnyForLaunch() {
        mode = Mode.LAUNCHER;
        if (isEmpty()) return false;
        activeSlot = getIndexOfClosestArtifact();
        return true;
    }

    /**
     * rotates the drum to align a prioritized artifact with the launcher. If there are no
     * prioritized artifacts, rotates to the other artifact color.
     * @return whether any artifact was found
     */
    public boolean alignPrioritizedForLaunch(Artifact artifact) {
        mode = Mode.LAUNCHER;
        boolean wasSuccess = alignForLaunch(artifact);
        if (!wasSuccess) wasSuccess = alignAnyForLaunch();
        return wasSuccess;
    }

    /**
     * rotates the drum to align an empty slot with the intake
     * @return whether an empty slot was found
     */
    public boolean alignForIntake() {
        mode = Mode.INTAKE;
        if (isFull()) return false;
        activeSlot = getIndexOfClosestArtifact(Artifact.EMPTY);
        return true;
    }

    public int getNumOfArtifact(Artifact artifact) {
        int result = 0;
        for (Artifact artifact1 : drum) {
            if (artifact1 == artifact) result++;
        }
        return result;
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

    public boolean isFull() {
        return getNumOfArtifact(Artifact.EMPTY) == 0;
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

    public DetectedColor getDetectedColor() {
        int rawColor = colorSensor.getNormalizedColors().toColor();
        if (HOME_COLOR_THRESHOLD.isColorInRange(rawColor)) {
            return DetectedColor.HOME_COLOR;
        } else if (GREEN_THRESHOLD.isColorInRange(rawColor)) {
            return DetectedColor.GREEN;
        } else if (PURPLE_THRESHOLD.isColorInRange(rawColor)) {
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }

    public void emptyActiveSlot() {
        drum[activeSlot] = Artifact.EMPTY;
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

    private double getDistanceFromIndex(int slotIndex) {
        double indexAngle = baseAngle + (slotIndex * 120);
        double currentAngle = getCurrentAngle();
        double distance = (currentAngle - indexAngle) % 360;

        // normalize distance between [-179.9 to 180]
        if (distance > 180) distance -= 360;
        if (distance <= -180) distance += 360;
        return distance;
    }

    /**
     * get current angle of the spindexer DcMotor in degrees
     */
    private double getCurrentAngle() {
        return MathEx.encoderPositionToDegrees(spindexerMotor.getCurrentPosition() + encoderOffset, ENCODER_RES);
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

    private Artifact getDetectedArtifact() {
        if (!isObjectInProximity()) return Artifact.EMPTY;

        switch (getDetectedColor()) {
            case GREEN:
                return Artifact.GREEN;
            case PURPLE:
                return Artifact.PURPLE;
            default:
                return Artifact.EMPTY;
        }
    }
}