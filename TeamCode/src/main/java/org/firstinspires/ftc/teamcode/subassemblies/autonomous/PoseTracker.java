package org.firstinspires.ftc.teamcode.subassemblies.autonomous;

import static org.firstinspires.ftc.teamcode.util.MathKt.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.GenericCam;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.ArrayList;
import java.util.List;

import javax.annotation.CheckForNull;

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

    PinpointOdo pinpointOdo;
    GenericCam genericCam;
    List<Localizer> localizers = new ArrayList<>();

    MecDriveBase driveBase;
    Underglow underglow;

    final Pose startingPose;
    Pose currentPose;
    Pose targetPose;

    // note use of multiple controllers, one is for accuracy (approach), the other for speed (drive)
    PDController xDrivePDController = new PDController(DRIVE_P, DRIVE_D);
    PDController yDrivePDController = new PDController(DRIVE_P, DRIVE_D);
    PIDController xApproachPIDController = new PIDController(APPROACH_P, APPROACH_I, APPROACH_D);
    PIDController yApproachPIDController = new PIDController(APPROACH_P, APPROACH_I, APPROACH_D);
    PIDController headingPIDController = new PIDController(HEADING_P, HEADING_I, HEADING_D);

    ControllerType controllerType;

    boolean isMovementEnabled = false;

    public PoseTracker(LinearOpMode opMode, Pose startingPose) {
        super(opMode, "PoseTracker");
        this.startingPose = startingPose;
        pinpointOdo = new PinpointOdo(opMode, this.startingPose);
        genericCam = new GenericCam(opMode);
        driveBase = new MecDriveBase(opMode);
        underglow = new Underglow(opMode);

        // whatever localizer has the lowest index will take precedent
        localizers.add(1, pinpointOdo);
        localizers.add(0, genericCam);
    }

    public void update() {

        currentPose = getPrioritizedPose();
        if (currentPose == null && isMovementEnabled) {
            RobotLog.w("(PoseTracker) currentPose is null, disabling autonomous movement and stopping robot");
            disableMovement();
            driveBase.stopMotors();
            underglow.setColor(Underglow.Color.ORANGE);
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
                RobotLog.e("(PoseTracker) Controller Type is null, setting it to APPROACH");
                xPower = 0.0;
                yPower = 0.0;
            }
            // for the heading PID, we need to account for the fact that headings wrap around at 180 degrees. There is a great explanation of this at: https://www.ctrlaltftc.com/practical-examples/controlling-heading
            // so, instead of giving the PIDController the current and target heading, we give it the error (which we find ourselves) and the targetError (0)
            double hError = AngleUnit.normalizeDegrees(currentPose.h - targetPose.h);
            double hPower = headingPIDController.calculate(hError, 0);

            xPower = clamp(xPower, -MAX_POWER, MAX_POWER);
            yPower = clamp(yPower, -MAX_POWER, MAX_POWER);
            hPower = clamp(hPower, -MAX_POWER, MAX_POWER);

            moveRobotFieldCentric(
                    USE_X ? xPower : 0,
                    USE_Y ? yPower : 0,
                    USE_H ? hPower : 0
            );
        }
    }

    /** this method returns the pose of the earliest nonnull pose value from the localizer list,
     * used to make sure more accurate or precise sensors are prioritized for localization. it will
     * also set less accurate sensors to the most prioritized pose*/
    @CheckForNull
    public Pose getPrioritizedPose() {
        for (int i = 0; i < localizers.size() - 1; i++) {
            localizers.get(i).update();
        }

        Pose pose = null;
        for (int i = 0; i < localizers.size() - 1; i++) {
            Localizer localizer = localizers.get(i);
            if (localizer.getPose() != null) {
                pose = localizer.getPose();
                break;
            }
        }

        if (pose != null) {
            for (int i = 0; i < localizers.size() - 1; i++) {
                localizers.get(i).setPose(pose);
            }
        }

        return pose;
    }

    public void stop() {
        Global.lastPose = currentPose;
    }

    public enum ControllerType {
        DRIVE, APPROACH
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
        double heading;
        if (Global.ANGLE_UNIT == AngleUnit.DEGREES) {
            heading = Math.toRadians(currentPose.h);
        } else {
            heading = currentPose.h;
        }

        // Rotate the movement direction to counter the bot's rotation
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