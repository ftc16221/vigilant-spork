package org.firstinspires.ftc.teamcode.subassemblies;

import static org.firstinspires.ftc.teamcode.util.MathKt.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.concurrent.TimeUnit;

/**
 * This class keeps track of the robot's position and handles all autonomous movement
 */
@Config
public class PoseTracker extends Subassembly {

    public static double driveP = 0.0, driveD = 0.0;
    public static double approachP = 0.0, approachI = 0.0, approachD = 0.0;
    public static double headingP = 0.0, headingI = 0.0, headingD = 0.0;
    public static boolean updateGainLive = false;
    public static double maxPower = 0.8;
    public static boolean useX = true, useY = true, useH = true;
    public static LocalizationMode localizationMode = LocalizationMode.HYBRID;
    public static int apriltagUpdateInterval = 500;

    Odometry odometry;
    MecDriveBase driveBase;
    Vision vision;

    final Pose startingPose;
    Pose currentPose;
    Pose targetPose;

    PDController xDrivePDController = new PDController(driveP, driveD);
    PDController yDrivePDController = new PDController(driveP, driveD);
    PIDController xApproachPIDController = new PIDController(approachP, approachI, approachD);
    PIDController yApproachPIDController = new PIDController(approachP, approachI, approachD);
    PIDController headingPIDController = new PIDController(headingP, headingI, headingD);

    Deadline apriltagUpdateDeadline = new Deadline(apriltagUpdateInterval, TimeUnit.MILLISECONDS);

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
        currentPose = odometry.getPose(); // TODO: support aprilTags

        // used for live tuning via FTC dashboard
        if (updateGainLive) {
            xDrivePDController.setP(driveP);
            xDrivePDController.setD(driveD);
            yDrivePDController.setP(driveP);
            yDrivePDController.setD(driveD);
            xApproachPIDController.setPID(approachP, approachI, approachD);
            yApproachPIDController.setPID(approachP, approachI, approachD);
            headingPIDController.setPID(headingP, headingI, headingD);
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

            xPower = clamp(xPower, -maxPower, maxPower);
            yPower = clamp(yPower, -maxPower, maxPower); // yPower should in fact be passed in as param x
            hPower = clamp(hPower, -maxPower, maxPower);

            moveRobotFieldCentric(
                    useX ? xPower : 0,
                    useY ? yPower : 0,
                    useH ? hPower : 0
            );
        }
    }

    public enum ControllerType {
        DRIVE, APPROACH
    }

    public enum LocalizationMode {
        ODOMETRY,
        HYBRID,
        APRILTAG
    }

    public void setControllerType(ControllerType controllerType) { this.controllerType = controllerType; }
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
}