package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
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

    private static final Artifact[] slots = new Artifact[3];

    private final DcMotor spindexerMotor;
    private final NormalizedColorSensor colorSensor;

    private double position = INTAKE_ANGLE;
    private int index = 0;


    public Spindexer(OpMode opMode) {
        super(opMode, "Spindexer");

        spindexerMotor = opMode.hardwareMap.dcMotor.get("spindexer");
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void start() {
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update() {
        Artifact detectedArtifact = getDetectedArtifact();
        if (!isFull() && position == INTAKE_ANGLE && !spindexerMotor.isBusy() && detectedArtifact != Artifact.EMPTY) {
            slots[index] = detectedArtifact;

            if (isFull()) {
                // go to first color of motif first, to save time
                if (Global.getInstance().motif.toString().startsWith("G") && contains(Artifact.GREEN)) {
                    goTo(LAUNCHER_ANGLE, getIndexOfType(Artifact.GREEN));
                } else {
                    goTo(LAUNCHER_ANGLE, getIndexOfType(Artifact.PURPLE));
                }
            } else {
                // get ready for another
                goTo(INTAKE_ANGLE, getIndexOfType(Artifact.EMPTY));
            }
        }
    }

    public enum DetectedItem {
        HOME, GREEN, PURPLE, UNKNOWN
    }

    public enum Artifact {
        GREEN, PURPLE, EMPTY
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

    public Artifact getDetectedArtifact() {
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

    private void goTo(double angle, int index) {
        this.position = angle;
        this.index = index;

        double targetAngle = angle + (index * 120);
        double currentAngle = getCurrentAngle();
        double error = (currentAngle - targetAngle) % 360;

        // normalize error between [-179.9 to 180]
        if (error > 180) error -= 360;
        if (error <= -180) error += 360;

        setTargetAngle(currentAngle - error); // 1725 + 135 = 1860
    }

    public int getNumOfType(Artifact type) {
        int result = 0;
        for (Artifact artifact : slots) {
            if (artifact == type) result++;
        }
        return result;
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

    public boolean isFull() {
        return getNumOfType(Artifact.EMPTY) == 0;
    }

    public int getIndexOfType(Artifact type) {
        int result = -1;
        for (int i = 0; i < 3; i++) {
            if (slots[i] == type) {
                result = i;
                break;
            }
        }
        return result;
    }

    /**
     * gets index + 1, accounting for wrapping
     */
    public int getNextIndex() {
        return (index + 1) % 3;
    }

    /**
     * returns the spindexer DcMotor
     */
    public DcMotor getMotor() {
        return spindexerMotor;
    }

    /**
     * set the target angle of the spindexer DcMotor in degrees
     */
    public void setTargetAngle(double targetAngle) {
        spindexerMotor.setTargetPosition(MathEx.degreesToEncoderPosition(targetAngle, ENCODER_RES));
    }

    /**
     * get current angle of the spindexer DcMotor in degrees
     */
    public double getCurrentAngle() {
        return MathEx.encoderPositionToDegrees(spindexerMotor.getCurrentPosition(), ENCODER_RES);
    }

    /**
     * checks and returns whether an object is within {@value PROXIMITY_THRESHOLD} cm of the color sensor
     */
    public boolean isObjectInProximity() {
        if (colorSensor instanceof DistanceSensor) {
            DistanceSensor distanceSensor = (DistanceSensor) colorSensor;
            return distanceSensor.getDistance(DistanceUnit.CM) <= PROXIMITY_THRESHOLD;
        } else {
            return true;
        }
    }
}