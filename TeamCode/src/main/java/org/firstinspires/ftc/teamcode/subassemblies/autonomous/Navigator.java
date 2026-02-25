package org.firstinspires.ftc.teamcode.subassemblies.autonomous;

import static org.firstinspires.ftc.teamcode.util.MathEx.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Subassembly;

/**
 * This class keeps track of the robot's position and handles all autonomous movement
 */
@Config
public class Navigator extends Subassembly {

    // TODO: find more accurate coefficients before competition on the proper surface (foam tiles)
    public static double DRIVE_P = 0.01, DRIVE_D = 0.0001;
    public static double APPROACH_P = 0.02, APPROACH_I = 0.03, APPROACH_D = 0.0001;
    public static double HEADING_P = 0.02, HEADING_I = 0.1, HEADING_D = 0.0035;

    public static double MAX_POWER = 1.0;
    public static boolean USE_X = true, USE_Y = true, USE_H = true;
    public static double LINEAR_APPROACH_TOLERANCE = 3, HEADING_APPROACH_TOLERANCE = 2;
    public static double LINEAR_DRIVE_TOLERANCE = 30, HEADING_DRIVE_TOLERANCE = 10;

    public static double TRACKING_OFFSET = -90; // degrees

    public boolean useAccurateTolerance = true;

    FtcDashboard dashboard;

    MecDriveBase driveBase;
    Underglow underglow;
    LocalizationManager localizationManager;

    Pose currentPose;
    Pose targetPose;
    Pose trackingPoint;

    PIDController xPIDController = new PIDController(APPROACH_P, APPROACH_I, APPROACH_D);
    PIDController yPIDController = new PIDController(APPROACH_P, APPROACH_I, APPROACH_D);
    PIDController headingPIDController = new PIDController(HEADING_P, HEADING_I, HEADING_D);

    final OpModeType opModeType;

    boolean isMovementEnabled = false;
    boolean isPointTrackingEnabled = false;
    double trackingPower = 0;

    public Navigator(OpMode opMode, LocalizationManager localizationManager) {
        super(opMode, "Navigator");
        this.localizationManager = localizationManager;
        dashboard = FtcDashboard.getInstance();
        driveBase = new MecDriveBase(opMode);
        underglow = new Underglow(opMode);

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

    }

    public void start() {
        localizationManager.start();
    }

    public void update() {

        localizationManager.update();
        currentPose = localizationManager.getPose();

        if (currentPose == null && isMovementEnabled) {
            Watchdog.e("(Navigator) currentPose is null, disabling autonomous movement and stopping robot");
            disableMovement();
            driveBase.stopMotors();
            underglow.setColor(Underglow.Color.ORANGE);
            return;
        }

        if (targetPose == null && isMovementEnabled) {
            Watchdog.e("(Navigator) targetPose is null, disabling autonomous movement and stopping robot");

            disableMovement();
            isPointTrackingEnabled = false;
            driveBase.stopMotors();
            underglow.setColor(Underglow.Color.ORANGE);
            return;
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

        // used for live tuning via FTC dashboard
        if (Global.ENABLE_TUNING_MODE) {
            xPIDController.setPID(APPROACH_P, APPROACH_I, APPROACH_D);
            yPIDController.setPID(APPROACH_P, APPROACH_I, APPROACH_D);
            headingPIDController.setPID(HEADING_P, HEADING_I, HEADING_D);

            TelemetryPacket packet = new TelemetryPacket();
            if (targetPose != null) {
                packet.put("xError", targetPose.x - currentPose.x);
                packet.put("yError", targetPose.y - currentPose.y);
                packet.put("hError", AngleUnit.normalizeDegrees(targetH - currentPose.h));
                dashboard.sendTelemetryPacket(packet);
            }
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
            double xPower = xPIDController.calculate(currentPose.x, targetPose.x);
            double yPower = yPIDController.calculate(currentPose.y, targetPose.y);

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

    public void runTelemetry() {
        telemetry.addLine("Navigator:");
        telemetry.addData("Is Movement Enabled", isMovementEnabled);
        telemetry.addData("Is Point Tracking Enabled", isPointTrackingEnabled);
        if (currentPose != null)
            telemetry.addData("Current Position", "x=%.1f, y=%.1f, h=%.1f°", currentPose.x, currentPose.y, currentPose.h);
        if (targetPose != null)
            telemetry.addData("Target Position", "x=%.1f, y=%.1f, h=%.1f°", targetPose.x, targetPose.y, targetPose.h);
        telemetry.addLine();
    }

    public boolean isAtTarget() {
        if (currentPose == null) return false;
        double xPosError = currentPose.x - targetPose.x;
        double yPosError = currentPose.y - targetPose.y;
        double hPosError = currentPose.h - targetPose.h;

        double linearError = Math.hypot(xPosError, yPosError);
        double headingError = Math.abs(hPosError);

        boolean withinTolerance;
        if (useAccurateTolerance) {
            withinTolerance = !localizationManager.isRobotMoving() && linearError < LINEAR_APPROACH_TOLERANCE && headingError < HEADING_APPROACH_TOLERANCE;
        } else {
            withinTolerance = linearError < LINEAR_DRIVE_TOLERANCE && headingError < HEADING_DRIVE_TOLERANCE;
        }
        return withinTolerance;
    }

    public void stop() {
        Global.lastPose = currentPose;
        localizationManager.stop();
    }

    private enum OpModeType {
        TELEOP, AUTONOMOUS, UNKNOWN
    }

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