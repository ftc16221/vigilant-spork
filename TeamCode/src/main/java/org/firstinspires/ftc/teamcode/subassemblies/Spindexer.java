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

    public static double INTAKE_ANGLE = 0.0; // degrees
    public static double LAUNCHER_ANGLE = 180.0; // degrees TODO: find actual value

    public static double ENCODER_RES = 384.5; // PPR

    // color thresholds are in 0xAARRGGBB formatting
    public static ColorThreshold HOME_COLOR_THRESHOLD = new ColorThreshold(0xFF9E5C29, 0xFFFCAD03); // orange
    public static ColorThreshold GREEN_THRESHOLD = new ColorThreshold(0xFF36572A, 0xFF03FC94);
    public static ColorThreshold PURPLE_THRESHOLD = new ColorThreshold(0xFF7855A3, 0xFFC200FF);

    public static float COLOR_SENSOR_GAIN = 2.0f; // always >=1
    public static double PROXIMITY_THRESHOLD = 5.0; // cm

    public static double kP = 0.0, kI = 0.0, kD = 0.0, kF = 0.0;
    public static int TOLERANCE = 2; // degrees

    private final Artifact[] drum = new Artifact[3];

    private final DcMotor spindexerMotor;
    private final NormalizedColorSensor colorSensor;
    private final PIDFController spindexerPIDF = new PIDFController(kP, kI, kD, kF);;

    private int targetPosition = 0;
    private double targetAngle = INTAKE_ANGLE;
    private int activeSlot = 0;


    public Spindexer(OpMode opMode) {
        super(opMode, "Spindexer");

        spindexerMotor = opMode.hardwareMap.dcMotor.get("spindexer");
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        align(0, 0);

        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        colorSensor.setGain(COLOR_SENSOR_GAIN);

        boolean isAutoOpMode = opMode.getClass().isAnnotationPresent(Autonomous.class);
        if (isAutoOpMode || drum[0] == null) {
            drum[0] = Artifact.GREEN;
            drum[1] = Artifact.PURPLE;
            drum[2] = Artifact.PURPLE;
        }
    }

    public void update() {

        if (Global.ENABLE_TUNING_MODE) {
            spindexerPIDF.setPIDF(kP, kI, kD, kF);
            colorSensor.setGain(COLOR_SENSOR_GAIN);
        }

        double power = spindexerPIDF.calculate(spindexerMotor.getCurrentPosition(), targetPosition);
        power = MathEx.clamp(power, -1, 1);
        sendData("spindexer power", power);
        spindexerMotor.setPower(power);

        // intake mode
        Artifact detectedArtifact = getDetectedArtifact();
        if (!isFull() && targetAngle == INTAKE_ANGLE && !isBusy() && detectedArtifact != Artifact.EMPTY) {
            drum[activeSlot] = detectedArtifact;

            if (isFull()) {
                // go to first color of motif first, to save time
                if (Global.motif != null && Global.motif.toString().startsWith("G") && contains(Artifact.GREEN)) {
                    alignPrioritizedForLaunch(Artifact.GREEN);
                } else {
                    alignPrioritizedForLaunch(Artifact.PURPLE);
                }
            } else {
                // get ready for another artifact
                alignForIntake();
            }
        }
    }

    public void stop() {
        spindexerMotor.setPower(0.0);
    }

    public enum DetectedItem {
        HOME, GREEN, PURPLE, UNKNOWN
    }

    public enum Artifact {
        GREEN, PURPLE, EMPTY
    }

    /**
     * rotates the drum to align a specified artifact color with launcher
     * @return whether an artifact of specified color was found
     */
    public boolean alignForLaunch(Artifact artifact) {
        if (!contains(artifact)) return false;
        align(LAUNCHER_ANGLE, getIndexOfClosestArtifact(artifact));
        drum[activeSlot] = Artifact.EMPTY;
        return true;
    }

    /**
     * rotates the drum to align an artifact of either color with launcher
     * @return whether any artifact was found
     */
    public boolean alignAnyForLaunch() {
        if (isEmpty()) return false;
        align(LAUNCHER_ANGLE, getIndexOfClosestArtifact());
        drum[activeSlot] = Artifact.EMPTY;
        return true;
    }

    /**
     * rotates the drum to align a prioritized artifact with the launcher. If there are no
     * prioritized artifacts, rotates to the other artifact color.
     * @return whether any artifact was found
     */
    public boolean alignPrioritizedForLaunch(Artifact artifact) {
        boolean wasSuccess = alignForLaunch(artifact);
        if (!wasSuccess) wasSuccess = alignAnyForLaunch();
        if (wasSuccess) drum[activeSlot] = Artifact.EMPTY;
        return wasSuccess;
    }

    /**
     * rotates the drum to align an empty slot with the intake
     * @return whether an empty slot was found
     */
    public boolean alignForIntake() {
        if (isFull()) return false;
        align(INTAKE_ANGLE, getIndexOfClosestArtifact(Artifact.EMPTY));
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
        int positionError = Math.abs(targetPosition - spindexerMotor.getCurrentPosition());
        double errorInDegrees = MathEx.encoderPositionToDegrees(positionError, ENCODER_RES);
        return errorInDegrees > TOLERANCE;
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

    public DetectedItem getDetectedColor() {
        int rawColor = colorSensor.getNormalizedColors().toColor();
        if (HOME_COLOR_THRESHOLD.isColorInRange(rawColor)) {
            return DetectedItem.HOME;
        } else if (GREEN_THRESHOLD.isColorInRange(rawColor)) {
            return DetectedItem.GREEN;
        } else if (PURPLE_THRESHOLD.isColorInRange(rawColor)) {
            return DetectedItem.PURPLE;
        } else {
            return DetectedItem.UNKNOWN;
        }
    }

    private int getIndexOfClosestArtifact(Artifact artifact) {
        int result = -1;
        double smallestDistance = 361;
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
        double smallestDistance = 361;
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
        double indexAngle = targetAngle + (slotIndex * 120);
        double currentAngle = getCurrentAngle();
        double distance = (currentAngle - indexAngle) % 360;

        // normalize distance between [-179.9 to 180]
        if (distance > 180) distance -= 360;
        if (distance <= -180) distance += 360;
        return distance;
    }

    private void align(double drumAngle, int slotIndex) {
        this.targetAngle = drumAngle;
        this.activeSlot = slotIndex;

        double distance = getDistanceFromIndex(slotIndex);

        setTargetAngle(getCurrentAngle() - distance);
    }

    /**
     * get current angle of the spindexer DcMotor in degrees
     */
    private double getCurrentAngle() {
        return MathEx.encoderPositionToDegrees(spindexerMotor.getCurrentPosition(), ENCODER_RES);
    }

    /**
     * set the target angle of the spindexer DcMotor in degrees
     */
    private void setTargetAngle(double targetAngle) {
        targetPosition = (MathEx.degreesToEncoderPosition(targetAngle, ENCODER_RES));
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