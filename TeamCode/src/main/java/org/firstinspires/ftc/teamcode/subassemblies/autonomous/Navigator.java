package org.firstinspires.ftc.teamcode.subassemblies.autonomous;

import static org.firstinspires.ftc.teamcode.util.LoggingKt.log;
import static org.firstinspires.ftc.teamcode.util.MathEx.clamp;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
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
public class Navigator extends Subassembly {

    // TODO: find more accurate coefficients before competition on the proper surface (foam tiles)
    public static double DRIVE_P = 0.01, DRIVE_D = 0.0001;
    public static double APPROACH_P = 0.01, APPROACH_I = 0.03, APPROACH_D = 0.0001;
    public static double HEADING_P = 0.05, HEADING_I = 0.1, HEADING_D = 0.0035;

    public static double MAX_POWER = 0.8;
    public static boolean USE_X = true, USE_Y = true, USE_H = true;
    public static double LINEAR_APPROACH_TOLERANCE = 3, HEADING_APPROACH_TOLERANCE = 2;
    public static double LINEAR_DRIVE_TOLERANCE = 30, HEADING_DRIVE_TOLERANCE = 10;

    public static double TRACKING_OFFSET = -90; // degrees

    FtcDashboard dashboard;

    public PinpointOdo pinpointOdo;
    LimelightCam limelightCam;
    List<Localizer> localizers = new ArrayList<>();
    Localizer activeLocalizer = null;

    MecDriveBase driveBase;
    Underglow underglow;

    final Pose startingPose;
    Pose currentPose;
    Pose targetPose;
    Pose trackingPoint;

    // note use of multiple controllers, one is for accuracy (approach), the other for speed (drive)
    PDController xDrivePDController = new PDController(DRIVE_P, DRIVE_D);
    PDController yDrivePDController = new PDController(DRIVE_P, DRIVE_D);
    PIDController xApproachPIDController = new PIDController(APPROACH_P, APPROACH_I, APPROACH_D);
    PIDController yApproachPIDController = new PIDController(APPROACH_P, APPROACH_I, APPROACH_D);
    PIDController headingPIDController = new PIDController(HEADING_P, HEADING_I, HEADING_D);

    ControllerType controllerType;
    final OpModeType opModeType;

    boolean isMovementEnabled = false;
    boolean isPointTrackingEnabled = false;
    double trackingPower = 0;

    public Navigator(OpMode opMode, Pose startingPose) {
        super(opMode, "Navigator");
        dashboard = FtcDashboard.getInstance();
        this.startingPose = startingPose;
        currentPose = startingPose;
        pinpointOdo = new PinpointOdo(opMode, this.startingPose);
        limelightCam = new LimelightCam(opMode);
        driveBase = new MecDriveBase(opMode);
        underglow = new Underglow(opMode);

        // whatever localizer has the lowest index will take precedent
        localizers.add(limelightCam);
        localizers.add(pinpointOdo);

        // we can assume that if the opMode is an Autonomous opMode that we can immediately enable movement. If we can't that should be explicitly disabled.
        Class<? extends OpMode> opModeClass = opMode.getClass();
        if (opModeClass.isAnnotationPresent(Autonomous.class)) {
            opModeType = OpModeType.AUTONOMOUS;
            Watchdog.i("(Navigator) OpMode appears to be Autonomous, automatically enabling movement");
            enableMovement();
        } else if (opModeClass.isAnnotationPresent(TeleOp.class)) {
            opModeType = OpModeType.TELEOP;
        } else {
            opModeType = OpModeType.UNKNOWN;
        }

        log(opMode, "Navigator successfully initialized with the following Localizers: " + localizers);
    }

    public void update() {

        currentPose = getPrioritizedPose();

        if (currentPose == null && isMovementEnabled) {
            Watchdog.e("(Navigator) currentPose is null, disabling autonomous movement and stopping robot");
            disableMovement();
            driveBase.stopMotors();
            underglow.setColor(Color.YELLOW);
            return;
        }
        assert currentPose != null;

        if (targetPose == null && isMovementEnabled) {
            Watchdog.e("(Navigator) targetPose is null, disabling autonomous movement and stopping robot");

            disableMovement();
            isPointTrackingEnabled = false;
            driveBase.stopMotors();
            underglow.setColor(Color.YELLOW);
            return;
        }
//        assert targetPose != null;

        if (Global.motif == null) {
            List<Integer> tagIds = limelightCam.getDetectedTagIds();
            if (tagIds.contains(21)) Global.motif = Global.Motif.GPP;
            else if (tagIds.contains(22)) Global.motif = Global.Motif.PGP;
            else if (tagIds.contains(23)) Global.motif = Global.Motif.PPG;
            if (Global.motif != null) {
                Watchdog.i(String.format("Pattern %s detected via obelisk apriltag", Global.motif.name()));
            }}

        // used for live tuning via FTC dashboard
        if (Global.ENABLE_TUNING_MODE) {
            xDrivePDController.setP(DRIVE_P);
            xDrivePDController.setD(DRIVE_D);
            yDrivePDController.setP(DRIVE_P);
            yDrivePDController.setD(DRIVE_D);
            xApproachPIDController.setPID(APPROACH_P, APPROACH_I, APPROACH_D);
            yApproachPIDController.setPID(APPROACH_P, APPROACH_I, APPROACH_D);
            headingPIDController.setPID(HEADING_P, HEADING_I, HEADING_D);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("xError", targetPose.x - currentPose.x);
            packet.put("yError", targetPose.y - currentPose.y);
            packet.put("hError", AngleUnit.normalizeDegrees(targetPose.h - currentPose.h));
            dashboard.sendTelemetryPacket(packet);
        }

        if (!(isPointTrackingEnabled || isMovementEnabled)) return;

        double targetH;
        if (isPointTrackingEnabled) {
            Pose referencePose;
            if (isMovementEnabled) referencePose = targetPose;
            else referencePose = currentPose;
            targetH = findTrackedHeading(referencePose);
        } else {
            targetH = targetPose.h;
        }

        // for the heading PID, we need to account for the fact that headings wrap around at 180 degrees. There is a great explanation of this at: https://www.ctrlaltftc.com/practical-examples/controlling-heading
        // so, instead of giving the PIDController the current and target heading, we give it the error (which we find ourselves) and the targetError (0)
        double hError = Global.ANGLE_UNIT == AngleUnit.DEGREES ? AngleUnit.normalizeDegrees(targetH - currentPose.h) : AngleUnit.normalizeRadians(targetH - currentPose.h);
        double hPower = headingPIDController.calculate(hError, 0);
        sendData("targetH", targetH);
        sendData("hPower", hPower);
        if (!USE_H) hPower = 0;

        if (isPointTrackingEnabled) trackingPower = hPower;

        if (isMovementEnabled) {

            double xPower, yPower;
            if (controllerType == ControllerType.APPROACH) {
                xPower = xApproachPIDController.calculate(currentPose.x, targetPose.x);
                yPower = yApproachPIDController.calculate(currentPose.y, targetPose.y);
            } else if (controllerType == ControllerType.DRIVE) {
                xPower = xDrivePDController.calculate(currentPose.x, targetPose.x);
                yPower = yDrivePDController.calculate(currentPose.y, targetPose.y);
            } else {
                RobotLog.e("(Navigator) Controller Type is null, setting it to APPROACH");
                controllerType = ControllerType.APPROACH;
                xPower = 0.0;
                yPower = 0.0;
            }

            xPower = clamp(xPower, -MAX_POWER, MAX_POWER);
            yPower = clamp(yPower, -MAX_POWER, MAX_POWER);
            hPower = clamp(hPower, -MAX_POWER, MAX_POWER);

            moveRobotFieldCentric(
                    USE_X ? xPower : 0,
                    USE_Y ? -yPower : 0, // TODO: negative because of some discrepancy somewhere with the standard field coordinates in moveRobotFieldCentric()
                    USE_H ? hPower : 0
            );

        }
    }

    private double findTrackedHeading(Pose referencePose) {
        Pose offsetPose = trackingPoint.subtract(referencePose);
        return Global.ANGLE_UNIT.fromRadians(Math.atan2(offsetPose.y, offsetPose.x)) + TRACKING_OFFSET; // heading that faces the targetPose
    }

    /** this method returns the pose of the earliest nonnull pose value from the localizer list,
     * used to make sure more accurate or precise sensors are prioritized for localization. it will
     * also set less accurate sensors to the most prioritized pose*/
    @CheckForNull
    public Pose getPrioritizedPose() {
        for (int i = 0; i <= localizers.size() - 1; i++) {
            localizers.get(i).update();
        }

        Pose pose = null;
        Localizer newActiveLocalizer = null;
        for (int i = 0; i <= localizers.size() - 1; i++) {
            Localizer localizer = localizers.get(i);
            if (localizer.getPose() != null) {
                pose = localizer.getPose();
                newActiveLocalizer = localizer;
                break;
            }
        }

        activeLocalizer = newActiveLocalizer;

        if (pose != null) {
            for (int i = 0; i <= localizers.size() - 1; i++) {
                Localizer localizer = localizers.get(i);
                if (localizer != newActiveLocalizer) localizer.setPose(pose);
            }
        }

        return pose;
    }

    public void runTelemetry() {
        telemetry.addLine("Pose Tracker:");
        String activeLocalizerString;
        if (activeLocalizer == null) activeLocalizerString = "NULL";
        else activeLocalizerString = activeLocalizer.getClass().getName();
        telemetry.addData("Active Localizer", activeLocalizerString);
        String currentControllerString;
        if (controllerType == ControllerType.APPROACH) currentControllerString = "APPROACH";
        else if (controllerType == ControllerType.DRIVE) currentControllerString = "DRIVE";
        else currentControllerString = "NULL";
        telemetry.addData("Current Controller", currentControllerString);
        telemetry.addData("Is Movement Enabled", isMovementEnabled);
        telemetry.addData("Is Point Tracking Enabled", isPointTrackingEnabled);
        if (currentPose != null)
            telemetry.addData("Current Position", "x=%.1f, y=%.1f, h=%.1f°", currentPose.x, currentPose.y, currentPose.h);
        if (targetPose != null)
            telemetry.addData("Target Position", "x=%.1f, y=%.1f, h=%.1f°", targetPose.x, targetPose.y, targetPose.h);
        telemetry.addLine();
    }

    public boolean isAtTarget() {
        double xPosError = currentPose.x - targetPose.x;
        double yPosError = currentPose.y - targetPose.y;
        double hPosError = currentPose.h - targetPose.h;
        double linearError = Math.hypot(xPosError, yPosError);
        double headingError = Math.abs(hPosError);
        boolean withinTolerance;
        if (controllerType == ControllerType.APPROACH) {
            withinTolerance = !activeLocalizer.isRobotMoving() && linearError < LINEAR_APPROACH_TOLERANCE && headingError < HEADING_APPROACH_TOLERANCE;
        } else if (controllerType == ControllerType.DRIVE) {
            withinTolerance = linearError < LINEAR_DRIVE_TOLERANCE && headingError < HEADING_DRIVE_TOLERANCE;
        } else {
            withinTolerance = false;
        }
        return withinTolerance;
    }

    public void stop() {
        Global.lastPose = currentPose;
    }

    public enum ControllerType {
        DRIVE, APPROACH
    }

    private enum OpModeType {
        TELEOP, AUTONOMOUS, UNKNOWN
    }
    
    /** Set which P(I)D controller to use */
    public void setControllerType(ControllerType controllerType) { this.controllerType = controllerType; }
    /** Get current P(I)D controller type */
    public ControllerType getControllerType() { return controllerType; }

    /** Set target pose, mirroring for blue alliance */
    public void setUnspecificTargetPose(Pose targetPose) {
        if (Global.alliance == Global.Alliance.BLUE) this.targetPose = targetPose.mirror();
        else this.targetPose = targetPose;
    }
    /** Set target pose ignoring alliance */
    public void setTargetPose(Pose targetPose) { this.targetPose = targetPose; }
    public Pose getTargetPose() { return targetPose; }
    public Pose getCurrentPose() { return currentPose; }

    /** Set tracking point, mirroring for blue alliance */
    public void setUnspecificTrackingPoint(Pose trackingPoint) {
        if (Global.alliance == Global.Alliance.BLUE) this.trackingPoint = trackingPoint.mirror();
        else this.trackingPoint = trackingPoint;
    }
    /** Set tracking point ignoring alliance */
    public void setTrackingPoint(Pose trackingPoint) { this.trackingPoint = trackingPoint; }
    public Pose getTrackingPoint() { return trackingPoint; }

    public double getTrackingPower() { return trackingPower; }

    public void enableMovement() { isMovementEnabled = true; }
    public void disableMovement() { isMovementEnabled = false; }
    public Boolean isMovementEnabled() { return isMovementEnabled; }

    public void enablePointTracking() { isPointTrackingEnabled = true; }
    public void disablePointTracking() { isPointTrackingEnabled = false; }
    public boolean isPointTrackingEnabled() { return isPointTrackingEnabled; }

    /**
     * Move robot according to desired axes motions.
     * Positive x is forward.
     * Positive y is strafe left.
     * Positive yaw is counter-clockwise.
     * <p>
     * see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">Field Centric Movement Breakdown</a>
     * or <a href="https://ftc-docs.firstinspires.org/en/latest/_images/decode-field.png">Coordinate System Definition</a>
     */
    public void moveRobotFieldCentric(double x, double y, double h) {
        double heading = Global.ANGLE_UNIT == AngleUnit.DEGREES ? Math.toRadians(currentPose.h) : currentPose.h;

        // Rotate the movement direction to counter the bot's rotation
        double rotX = y * Math.cos(-heading) - x * Math.sin(-heading);
        double rotY = y * Math.sin(-heading) + x * Math.cos(-heading);

        driveBase.moveRobot(rotX, rotY, h);
    }
}