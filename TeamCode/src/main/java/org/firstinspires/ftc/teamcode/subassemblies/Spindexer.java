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
    public static int TOLERANCE = 4; // PPR

    private final Artifact[] slots = new Artifact[3];

    private final DcMotor spindexerMotor;
    private final NormalizedColorSensor colorSensor;
    private final PIDFController spindexerPIDF = new PIDFController(kP, kI, kD, kF);;

    private int targetPosition = 0;
    private double targetAngle = INTAKE_ANGLE;
    private int index = 0;


    public Spindexer(OpMode opMode) {
        super(opMode, "Spindexer");

        spindexerMotor = opMode.hardwareMap.dcMotor.get("spindexer");
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goTo(0, 0);

        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        colorSensor.setGain(COLOR_SENSOR_GAIN);

        boolean isAutoOpMode = opMode.getClass().isAnnotationPresent(Autonomous.class);
        if (isAutoOpMode || slots[0] == null) {
            slots[0] = Artifact.GREEN;
            slots[1] = Artifact.PURPLE;
            slots[2] = Artifact.PURPLE;
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
            slots[index] = detectedArtifact;

            if (isFull()) {
                // go to first color of motif first, to save time
                if (Global.motif != null && Global.motif.toString().startsWith("G") && contains(Artifact.GREEN)) {
                    outtakePrioritized(Artifact.GREEN);
                } else {
                    outtakePrioritized(Artifact.PURPLE);
                }
            } else {
                // get ready for another artifact
                goTo(INTAKE_ANGLE, getIndexOfClosestArtifact(Artifact.EMPTY));
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
    public boolean outtake(Artifact artifact) {
        if (!contains(artifact)) return false;

        goTo(LAUNCHER_ANGLE, getIndexOfClosestArtifact(artifact));
        return true;
    }

    /**
     * rotates the drum to align an artifact of either color with launcher
     * @return whether any artifact was found
     */
    public boolean outtakeAny() {
        if (isEmpty()) return false;

        goTo(LAUNCHER_ANGLE, getIndexOfClosestArtifact());
        return true;
    }

    /**
     * rotates the drum to align a prioritized artifact with the launcher. If there are no
     * prioritized artifacts, rotates to the other artifact color.
     * @return whether any artifact was found
     */
    public boolean outtakePrioritized(Artifact artifact) {
        boolean wasSuccess = outtake(artifact);
        if (!wasSuccess) wasSuccess = outtakeAny();
        return wasSuccess;
    }

    public int getNumOfArtifact(Artifact artifact) {
        int result = 0;
        for (Artifact artifact1 : slots) {
            if (artifact1 == artifact) result++;
        }
        return result;
    }

    public boolean isBusy() {
        int error = Math.abs(targetPosition - spindexerMotor.getCurrentPosition());
        return error > TOLERANCE;
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

    /**
     * Warning: will output -1 if no artifact exists
     */
    private int getIndexOfClosestArtifact(Artifact artifact) {
        int result = -1;
        double smallestDistance = 361;
        for (int i = 0; i < 3; i++) {
            if (slots[i] == artifact) {
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
     * Warning: will output -1 if no artifact exists
     */
    private int getIndexOfClosestArtifact() {
        int result = -1;
        double smallestDistance = 361;
        for (int i = 0; i < 3; i++) {
            if (slots[i] != Artifact.EMPTY) {
                double distance = Math.abs(getDistanceFromIndex(i));
                if (distance < smallestDistance) {
                    smallestDistance = distance;
                    result = i;
                }
            }
        }
        return result;
    }

    private double getDistanceFromIndex(int index) {
        double indexAngle = targetAngle + (index * 120);
        double currentAngle = getCurrentAngle();
        double distance = (currentAngle - indexAngle) % 360;

        // normalize distance between [-179.9 to 180]
        if (distance > 180) distance -= 360;
        if (distance <= -180) distance += 360;
        return distance;
    }

    private void goTo(double angle, int index) {
        this.targetAngle = angle;
        this.index = index;

        double distance = getDistanceFromIndex(index);

        setTargetAngle(getCurrentAngle() - distance);
    }

    public boolean contains(Artifact type) {
        boolean result = false;
        for (Artifact artifact : slots) {
            if (artifact == type) {
                result = true;
                break;
            }
        }
        return result;
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

    private DetectedItem getDetectedColor() {
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