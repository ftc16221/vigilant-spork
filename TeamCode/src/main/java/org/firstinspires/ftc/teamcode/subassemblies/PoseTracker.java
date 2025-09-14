package org.firstinspires.ftc.teamcode.subassemblies;

import static org.firstinspires.ftc.teamcode.util.MathKt.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.concurrent.TimeUnit;

/**
 * This class keeps track of the robot's position and handles all autonomous movement
 */
@Config
public class PoseTracker extends Subassembly {

    public static double DRIVE_P = 0.0, DRIVE_D = 0.0;
    public static double APPROACH_P = 0.0, APPROACH_I = 0.0, APPROACH_D = 0.0;
    public static double HEADING_P = 0.0, HEADING_I = 0.0, HEADING_D = 0.0;
    public static boolean UPDATE_GAIN_LIVE = false;
    public static double MAX_POWER = 0.8;
    public static boolean USE_X = true, USE_Y = true, USE_H = true;
    public static int APRILTAG_UPDATE_INTERVAL = 500;
    public static LocalizationMode localizationMode = LocalizationMode.HYBRID;

    Odometry odometry;
    MecDriveBase driveBase;
    Vision vision;

    final Pose startingPose;
    Pose currentPose;
    Pose targetPose;

    // note use of multiple controllers, one is for accuracy (approach), the other for speed (drive)
    PDController xDrivePDController = new PDController(DRIVE_P, DRIVE_D);
    PDController yDrivePDController = new PDController(DRIVE_P, DRIVE_D);
    PIDController xApproachPIDController = new PIDController(APPROACH_P, APPROACH_I, APPROACH_D);
    PIDController yApproachPIDController = new PIDController(APPROACH_P, APPROACH_I, APPROACH_D);
    PIDController headingPIDController = new PIDController(HEADING_P, HEADING_I, HEADING_D);

    Deadline apriltagUpdateDeadline = new Deadline(APRILTAG_UPDATE_INTERVAL, TimeUnit.MILLISECONDS);

    ControllerType controllerType;

    boolean isMovementEnabled = false;

    public PoseTracker(LinearOpMode opMode, Pose startingPose) {
        super(opMode, "PoseTracker");
        this.startingPose = startingPose;
        odometry = new Odometry(opMode, this.startingPose);
        driveBase = new MecDriveBase(opMode);
        vision = new Vision(opMode);
    }

    public void update() {

        switch (localizationMode) {
            case ODOMETRY:
                currentPose = odometry.getPose();
                break;
            case HYBRID:
                Pose visionPose = vision.getPose();
                if (!odometry.isRobotMoving() && vision.getPositionIsNotNull() && apriltagUpdateDeadline.hasExpired()) {
                    apriltagUpdateDeadline.reset();
                    odometry.setPose(visionPose);
                    currentPose = visionPose;
                    if (vision.getValidDetections() != null)
                        RobotLog.i("(Follower) Updated Current Position based off AprilTag Detection (ID: " + vision.getValidDetections().get(0).id + ")");
                } else {
                    currentPose = odometry.getPose();
                }
                break;
            case APRILTAG:
                currentPose = vision.getPose();
                break;
        }

        drawFieldPosition();

        // used for live tuning via FTC dashboard
        if (UPDATE_GAIN_LIVE) {
            xDrivePDController.setP(DRIVE_P);
            xDrivePDController.setD(DRIVE_D);
            yDrivePDController.setP(DRIVE_P);
            yDrivePDController.setD(DRIVE_D);
            xApproachPIDController.setPID(APPROACH_P, APPROACH_I, APPROACH_D);
            yApproachPIDController.setPID(APPROACH_P, APPROACH_I, APPROACH_D);
            headingPIDController.setPID(HEADING_P, HEADING_I, HEADING_D);
        }

        if (isMovementEnabled) {
            double xPower = 0, yPower = 0;

            if (controllerType == ControllerType.APPROACH) {
                xPower = xApproachPIDController.calculate(currentPose.x, targetPose.x);
                yPower = yApproachPIDController.calculate(currentPose.y, targetPose.y);

            } else if (controllerType == ControllerType.DRIVE) {
                xPower = xDrivePDController.calculate(currentPose.x, targetPose.x);
                yPower = yDrivePDController.calculate(currentPose.y, targetPose.y);

            } else {
                RobotLog.e("(PoseTracker) Controller Type is null");
            }
            double hPower = headingPIDController.calculate(currentPose.h, targetPose.h);

            xPower = clamp(xPower, -MAX_POWER, MAX_POWER);
            yPower = clamp(yPower, -MAX_POWER, MAX_POWER); // yPower should in fact be passed in as param x
            hPower = clamp(hPower, -MAX_POWER, MAX_POWER);

            moveRobotFieldCentric(
                    USE_X ? xPower : 0,
                    USE_Y ? yPower : 0,
                    USE_H ? hPower : 0
            );
        }
    }

    public void stop() {
        Global.lastPose = currentPose;
    }

    public enum ControllerType {
        DRIVE, APPROACH
    }

    public enum LocalizationMode {
        ODOMETRY,
        HYBRID,
        APRILTAG
    }

    /** Set which P(I)D controller to use */
    public void setControllerType(ControllerType controllerType) { this.controllerType = controllerType; }
    /** Get current P(I)D controller type */
    public ControllerType getControllerType() { return controllerType; }

    public void setTargetPose(Pose targetPose) { this.targetPose = targetPose; }
    public Pose getTargetPose() { return targetPose; }
    public Pose getCurrentPose() { return currentPose; }

    public void enableMovement() { isMovementEnabled = true; }
    public void disableMovement() { isMovementEnabled = false; }
    public Boolean isMovementEnabled() { return isMovementEnabled; }

    /**
     * Move robot according to desired axes motions.
     * Positive x is forward.
     * Positive y is strafe left.
     * Positive yaw is counter-clockwise.
     * <p>
     * Field Centric Movement
     * see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">...</a>
     */
    private void moveRobotFieldCentric(double y, double x, double h) {
        double heading = Math.toRadians(currentPose.h);
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        driveBase.moveRobot(rotX, rotY, h);
    }

    /** Draws the robot's position onto the field in FTC Dashboard */
    private void drawFieldPosition() {
        TelemetryPacket packet = new TelemetryPacket(true);
        if (currentPose != null) {
            packet.fieldOverlay()
                    .setStroke("#12C600")
                    .setRotation(Math.toRadians(currentPose.h))
                    .setTranslation(currentPose.y, -currentPose.x) // x and y are swapped because FTC dash's coordinate system wants to be different
                    .strokeCircle(0, 0, 9) // draw circle for robot position
                    .strokeLine(0, 0, 9, 0);
        } else {
            packet.fieldOverlay().clear();
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}