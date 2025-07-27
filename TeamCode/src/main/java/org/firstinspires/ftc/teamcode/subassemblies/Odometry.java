package org.firstinspires.ftc.teamcode.subassemblies;

import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Subassembly;

public class Odometry extends Subassembly {

    public static double TRACK_WIDTH = 0; // distance between the middle of the wheels TODO: find val
    public static double ODO_ENCODER_RES = 0; // ticks per rev TODO: find val
    public static double ODO_RADIUS = 0; // radius of odometer wheels TODO: find val

    private Pose pose;

    private final LinearOpMode opMode;
    private final HardwareMap hardwareMap;

    private final DcMotorEx leftOdoPod, centerOdoPod, rightOdoPod; // we use motorEx bc the ftc sdk doesn't have a dedicated Encoder class, but these will only be used for encoder functionality
    private double lastLeftPos, lastRightPos, lastCenterPos;

    
    public Odometry(LinearOpMode opMode, Pose startingPose) {
        super(opMode, "Odometry");
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;

        this.pose = startingPose;

        leftOdoPod = hardwareMap.get(DcMotorEx.class, "left_odo_pod");
        centerOdoPod = hardwareMap.get(DcMotorEx.class, "center_odo_pod");
        rightOdoPod = hardwareMap.get(DcMotorEx.class, "right_odo_pod");
    }

    public void update() {
        double leftPos = leftOdoPod.getCurrentPosition() / ODO_ENCODER_RES * ODO_RADIUS;
        double centerPos = centerOdoPod.getCurrentPosition() / ODO_ENCODER_RES * ODO_RADIUS;
        double rightPos = rightOdoPod.getCurrentPosition() / ODO_ENCODER_RES * ODO_RADIUS;
        
        double leftPosChange = leftPos - lastLeftPos;
        double centerPosChange = centerPos - lastCenterPos;
        double rightPosChange = rightPos - lastRightPos;

        lastLeftPos = leftPos;
        lastCenterPos = centerPos;
        lastRightPos = rightPos;

        double forwardChange = (leftPosChange - rightPosChange) / 2; // checks how much bot has moved forward/backward, turning should cancel out
        double rotationalChange = (leftPosChange + rightPosChange) / 2; // checks how much bot has rotated in linear units, forward/backward movement should cancel out
        double lateralChange = centerPosChange - rotationalChange; // movement of center odoPod excluding rotation

        double headingChange = rotationalChange / (TRACK_WIDTH * PI ) * 360; // compares the arc (rotationalChange) to total circumfrence of circle in order to find the angle measure (headingChange)

        // convert lateral and forward robot-centric values to field centric coordinates
        pose.h += headingChange;
        double hRads = toRadians(pose.h); // radians

        double xChange = lateralChange * cos(hRads) - forwardChange * sin(hRads);
        double yChange = lateralChange * sin(hRads) + forwardChange * cos(hRads);

        pose.x += xChange;
        pose.y += yChange;

    }

    public Pose getPose() { return pose; }
}