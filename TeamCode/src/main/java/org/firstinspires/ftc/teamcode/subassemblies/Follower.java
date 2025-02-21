package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.AdvPose;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.List;

/**
 * Subassembly to autonomously move the drivebase
 * <p>
 * Javadoc comments partially written by Github Copilot, so if they don't make sense \_(ツ)_/ <sub>(thx copilot for that weird face)</sub>
 */
@Config
public class Follower extends Subassembly {

    // enable or disable position correction: useful for debugging
    public static boolean USE_X = true;
    public static boolean USE_Y = true;
    public static boolean USE_HEADING = true;

    public static double DRIVE_GAIN = 0.07; // Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error. (0.50 / 25.0)
    public static double TURN_GAIN = 0.04; // Turn Control "Gain". e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double MAX_AUTO_SPEED = 0.4; // Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_TURN = 0.2; // Clip the turn speed to this max value (adjust for your robot)

    public static SparkFunOTOS.Pose2D OTOS_OFFSET = new SparkFunOTOS.Pose2D(0, 0, 180);
    public static double OTOS_LINEAR_SCALAR = 1.01253481894;
    public static double OTOS_ANGULAR_SCALAR = 1;

    public static LocalizationMode LOCALIZATION_MODE = LocalizationMode.OTOS;

    private final LinearOpMode opMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final MecDriveBase driveBase;

    private final DcMotor leftRear;
    private final DcMotor leftFront;
    private final DcMotor rightRear;
    private final DcMotor rightFront;
    private final SparkFunOTOS OTOS; // Optical Tracking Odometry Sensor
    private final Vision vision;

    OpModeType opModeType;
    SparkFunOTOS.Pose2D startingPosition;
    SparkFunOTOS.Pose2D velocity;
    SparkFunOTOS.Pose2D currentPose;
    AdvPose targetPose;
    List<AdvPose> path;
    boolean enabled = true;
    boolean usePaths = true;
    double tolerance = 2;
    int pathState = 0;
    boolean busy = false;

    double currentH;
    double yDrive;
    double xDrive;
    double hDrive;
    double xError;
    double yError;
    double hError;

    /**
     * Constructor for Follower
     * @param opMode LinearOpMode object
     * @param startingPosition The starting position of the robot
     */
    public Follower(LinearOpMode opMode, SparkFunOTOS.Pose2D startingPosition) {
        super(opMode, "Follower");
        this.opMode = opMode;
        if (startingPosition == null) {
            RobotLog.w("(Follower) Starting position not set, disabling autonomous movement");
            disable();
            this.startingPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        } else {
            this.startingPosition = startingPosition;
        }
        hardwareMap = opMode.hardwareMap;
        telemetry = getTelemetry();

        driveBase = new MecDriveBase(opMode);

        leftRear = driveBase.getLeftRear();
        leftFront = driveBase.getLeftFront();
        rightRear = driveBase.getRightRear();
        rightFront = driveBase.getRightFront();

        OTOS = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOTOS(startingPosition);

        vision = new Vision(opMode);

        if (opMode.getClass().isAnnotationPresent(Autonomous.class)) {
            opModeType = OpModeType.AUTONOMOUS;
        } else if (opMode.getClass().isAnnotationPresent(TeleOp.class)) {
            opModeType = OpModeType.TELEOP;
        } else {
            opModeType = OpModeType.UNKNOWN;
        }

    }

    /**
     * Enable Autonomous Movement
     * <p>
     * Enabled by default
     */
    public void enable() {
        enabled = true;
    }

    /**
     * Disable Autonomous Movement
     * <p>
     * Should only be disabled during TeleOp
     */
    public void disable() {
        enabled = false;
    }

    /**
     * Check if Autonomous Movement is enabled
     * @return if Autonomous Movement is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Set the method of localization for the robot
     * <p>
     * OTOS selected by default
     */
    public void setLocalizationMode(LocalizationMode mode) {
        LOCALIZATION_MODE = mode;
    }

    public void setPath(List<AdvPose> path) {
        this.path = path;
    }

    public int getPathState() {
        return pathState;
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
    }

    public boolean isBusy() {
        return busy;
    }

    public void setTargetPose(AdvPose targetPose) {
        this.targetPose = targetPose;
    }

    public void update() {
        if (targetPose == null) {
            return;
        }
        updatePose();
        if (usePaths) {
            if (-1 < pathState && pathState < path.size() - 1) {
                targetPose = path.get(pathState);
            } else {
                return;
            }
        }

        // Get the x value from the given SparkFunOTOS.Pose2D object.
        double xError = currentPose.x - targetPose.x;
        double yError = currentPose.y - targetPose.y;
        double hError = (currentPose.h - targetPose.h) * (Math.abs(currentPose.h - targetPose.h) >= 180 ? -1 : 1);

        double xDrive = Range.clip(-xError * DRIVE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED); // why is this negative error?
        double yDrive = Range.clip(-yError * DRIVE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED); // eh i mean it works
        double hDrive = Range.clip(hError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN); // maybe

        if (enabled) {
            moveRobot(
                    USE_Y ? yDrive : 0,
                    USE_X ? xDrive : 0,
                    USE_HEADING ? hDrive : 0
            );
        }

        telemetry.addLine("Error values:");
        telemetry.addData("X error", xError);
        telemetry.addData("Y error", yError);
        telemetry.addData("H error", hError);
        telemetry.addLine("Drive values:");
        telemetry.addData("X drive", xDrive);
        telemetry.addData("Y drive", yDrive);
        telemetry.addData("H drive", hDrive);
        callTelemetry();

        if (!usePaths) {
            return;
        }

        if (robotInTolerance() && (!targetPose.stopAtEnd || !robotIsMoving())) {
            busy = false;
            moveRobot(0, 0, 0);
            opMode.sleep(100);
            if (!targetPose.hold) {
                // progress through path
                if (pathState >= path.size() - 1) {
                    pathState = -1;
                } else {
                    pathState++;
                }
            }
        } else {
            busy = true;
        }

    }

    /**
     * Gets the relevant pose of the robot based off of the localization mode.
     */
    public void updatePose() {
        switch (LOCALIZATION_MODE) {
            case OTOS:
                currentPose = OTOS.getPosition();
                break;
            case HYBRID:
                if (!robotIsMoving() && vision.getPosition() != null) {
                    OTOS.setPosition(vision.getPosition());
                }
                currentPose = OTOS.getPosition();
            case APRILTAG:
                currentPose = vision.getPosition();
                break;
        }
    }

    /**
     * Positive y is forward.
     * Positive x is strafe right.
     * Positive heading is counter-clockwise.
     * @version 1.3
     */
    public void driveToPose(SparkFunOTOS.Pose2D targetPose, double tolerance, boolean holdEnd) {
        this.tolerance = tolerance;
        currentPose = OTOS.getPosition();
        // Get the x value from the given SparkFunOTOS.Pose2D object.
        currentH = currentPose.h;
        // Get the x value from the given SparkFunOTOS.Pose2D object.
        xError = currentPose.x - targetPose.x;
        // Get the x value from the given SparkFunOTOS.Pose2D object.
        yError = currentPose.y - targetPose.y;
        hError = (currentPose.h - targetPose.h) * (Math.abs(currentPose.h - targetPose.h) >= 180 ? -1 : 1);
        while ((!robotInTolerance() || (holdEnd && robotIsMoving())) && opMode.opModeIsActive()) {
            currentPose = OTOS.getPosition();
            // Get the x value from the given SparkFunOTOS.Pose2D object.
            currentH = currentPose.h;
            // Determine y, x, and heading error so we can use them to control the robot automatically.
            // Basic error correction based on just the P of PID
            xError = currentPose.x - targetPose.x;
            yError = currentPose.y - targetPose.y;
            // Reverse the error if needed to allow for wrapping.
            hError = (currentH - targetPose.h) * (Math.abs(currentH - targetPose.h) >= 180 ? -1 : 1);
            // Use the speed and turn "gains" to calculate how we want the robot to move. Clip it to the maximum.
            yDrive = Range.clip(-yError * DRIVE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            xDrive = Range.clip(-xError * DRIVE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            hDrive = Range.clip(hError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            moveRobot(
                    USE_Y ? yDrive : 0,
                    USE_X ? xDrive : 0,
                    USE_HEADING ? hDrive : 0
            );
            callTelemetry();
        }
        moveRobot(0, 0, 0);
        opMode.sleep(100);
    }

    /**
     * Find the type of OpMode that is being run, based off the annotation
     * <p>
     * I know there is a reason I wrote this but I forgot
     * <p>
     * OHH it's so we don't use paths during teleop
     * @return type of OpMode (AUTONOMOUS, TELEOP)
     */
    private OpModeType getOpModeType() {
        if (opMode.getClass().isAnnotationPresent(Autonomous.class)) {
            return OpModeType.AUTONOMOUS;
        } else if (opMode.getClass().isAnnotationPresent(TeleOp.class)) {
            usePaths = false;
            return OpModeType.TELEOP;
        } else {
            return OpModeType.UNKNOWN;
        }
    }

    /**
     * Move robot according to desired axes motions.
     * Positive x is forward.
     * Positive y is strafe left.
     * Positive yaw is counter-clockwise.
     * <p>
     * Field Centric Movement
     * see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">...</a>
     */
    private void moveRobot(double y, double x, double h) {
        double rotX;
        double rotY;
        double max;
        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;

        currentPose = OTOS.getPosition();
        // Get the x value from the given SparkFunOTOS.Pose2D object.
        currentH = currentPose.h;
        // Rotate the movement direction counter to the bot's rotation
        rotX = x * Math.cos(-currentH / 180 * Math.PI) - y * Math.sin(-currentH / 180 * Math.PI);
        rotY = x * Math.sin(-currentH / 180 * Math.PI) + y * Math.cos(-currentH / 180 * Math.PI);
        // Normalize wheel powers to be less than 1.0.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(rotX), Math.abs(rotY), Math.abs(h), 1));
        // Calculate wheel powers.
        leftFrontPower = (rotY + rotX + h) / max;
        leftRearPower = ((rotY - rotX) + h) / max;
        rightFrontPower = ((rotY - rotX) - h) / max;
        rightRearPower = ((rotY + rotX) - h) / max;
        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    /**
     * Check if the robot is within the tolerance of the target position.
     * @return if robot is within tolerance
     */
    private boolean robotInTolerance() {
        boolean xInTolerance;
        boolean yInTolerance;
        boolean hInTolerance;

        xInTolerance = USE_X ? Math.abs(xError) < tolerance : true;
        yInTolerance = USE_Y ? Math.abs(yError) < tolerance : true;
        hInTolerance = USE_HEADING ? Math.abs(hError) < tolerance : true;
        telemetry.addData("X in tolerance", xInTolerance);
        telemetry.addData("Y in tolerance", yInTolerance);
        telemetry.addData("Heading in tolerance", hInTolerance);
        return xInTolerance && yInTolerance && hInTolerance;
    }

    /**
     * Check if the robot is moving.
     * @return if robot is moving
     */
    private boolean robotIsMoving() {
        boolean xIsMoving;
        boolean yIsMoving;
        boolean hIsMoving;

        velocity = OTOS.getVelocity();
        // Get the x value from the given SparkFunOTOS.Pose2D object.
        xIsMoving = Math.abs(velocity.x) > 2;
        // Get the x value from the given SparkFunOTOS.Pose2D object.
        yIsMoving = Math.abs(velocity.y) > 2;
        // Get the x value from the given SparkFunOTOS.Pose2D object.
        hIsMoving = Math.abs(velocity.h) > 1;
        return xIsMoving || yIsMoving || hIsMoving;
    }

    private void callTelemetry() {
        telemetry.addData("Auto", "yDrive" + JavaUtil.formatNumber(yDrive, 5, 2) + ", xDrive" + JavaUtil.formatNumber(xDrive, 5, 2) + ", turn" + JavaUtil.formatNumber(hDrive, 5, 2));
        telemetry.addLine("Coordinates:");
        telemetry.addData("X coordinate", JavaUtil.formatNumber(currentPose.x, 2));
        telemetry.addData("Y coordinate", JavaUtil.formatNumber(currentPose.y, 2));
        telemetry.addData("Heading angle", JavaUtil.formatNumber(currentPose.h, 2));
        telemetry.addData("Robot is moving", robotIsMoving());
        telemetry.addLine("Velocity:");
        velocity = OTOS.getVelocity();
        telemetry.addData("X velocity", JavaUtil.formatNumber(velocity.x, 2));
        telemetry.addData("Y velocity", JavaUtil.formatNumber(velocity.y, 2));
        telemetry.addData("Heading velocity", JavaUtil.formatNumber(velocity.h, 2));
        telemetry.addData("Robot in tolerance", robotInTolerance());
        // Update the telemetry on the driver station.
        telemetry.update();
    }

    /**
     * Configures the SparkFun OTOS
     */
    private void configureOTOS(SparkFunOTOS.Pose2D startingPosition) {
        SparkFunOTOS.Version hwVersion;
        SparkFunOTOS.Version fwVersion;

        telemetry.addLine("Configuring OTOS...");
        telemetry.update();
        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, so you need to set at the start of all your OpModes.
        OTOS.setLinearUnit(DistanceUnit.INCH);
        OTOS.setAngularUnit(AngleUnit.DEGREES);
        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        OTOS.setOffset(OTOS_OFFSET);
        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        OTOS.setLinearScalar(OTOS_LINEAR_SCALAR);
        OTOS.setAngularScalar(OTOS_ANGULAR_SCALAR);
        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu, you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total.
        OTOS.calibrateImu();
        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors.
        OTOS.resetTracking();
        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        OTOS.setPosition(startingPosition);
        // Get the hardware and firmware version
        hwVersion = new SparkFunOTOS.Version();
        fwVersion = new SparkFunOTOS.Version();
        OTOS.getVersionInfo(hwVersion, fwVersion);
        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine("");
        telemetry.addLine("OTOS Hardware Version: v" + hwVersion.major + "." + hwVersion.minor);
        telemetry.addLine("OTOS Firmware Version: v" + fwVersion.major + "." + fwVersion.minor);
        telemetry.update();
    }

    public enum OpModeType {
        TELEOP,
        AUTONOMOUS,
        UNKNOWN
    }

    public enum LocalizationMode {
        OTOS,
        HYBRID,
        APRILTAG
    }
}


/*
Here is old reference code to score the high basket autonomously

public void runOpMode() {

    configureOTOS(new SparkFunOTOS.Pose2D(-61.8, 36, 0));
    altClaw.scaleRange(0.1, 0.8);
    linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    opMode.sleep(10);
    linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    opMode.waitForStart();
    if (opMode.opModeIsActive()) {
        // Put run blocks here.
        altClaw.setPosition(1);
        altRotate.setPosition(0.8);

        raiseSlidesForSpecimen();
        driveToPos(
                -47,
                61,
                -135,
                2,
                true);

        scoreSpecimen();
        driveToPos(
                -48,
                36,
                -135,
                10,
                false);
        driveToPos(
                -12,
                36,
                0,
                10,
                false);
        ascend();
        driveToPos(
                -12,
                21.2,
                0,
                2,
                true);
    }
}

private void scoreSpecimen() {
    altRotate.setPosition(0.8);
    linearSlide.setTargetPosition((int) (SLIDE_ENCODER_RES * (LOW_BASKET_POS / (SLIDE_GEAR_DIAMETER * Math.PI))));
    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    linearSlide.setPower(1);
    while (linearSlide.isBusy() && opMode.opModeIsActive()) {
        telemetry.addData("linear slide pos", linearSlide.getCurrentPosition());
    }
    linearSlide.setPower(1);
    altRotate.setPosition(0.4);
    opMode.sleep(500);
    altClaw.setPosition(0);
    opMode.sleep(500);
    altRotate.setPosition(0.8);
    opMode.sleep(500);
    linearSlide.setTargetPosition(ASCEND_POS);
}

private void ascend() {
    linearSlide.setTargetPosition((int) (SLIDE_ENCODER_RES * (ASCEND_POS / (SLIDE_GEAR_DIAMETER * Math.PI))));
    linearSlide.setPower(15);
    while (linearSlide.isBusy() && opMode.opModeIsActive()) {
    }
    leftFront.setPower(0);
}
*/