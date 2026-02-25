package org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;

//@Config
public class ThreeWheelOdo extends Localizer {

    public static double TRACK_WIDTH = 0; // distance between the middle of the wheels TODO: find val
    public static double ODO_ENCODER_RES = 0; // ticks per rev TODO: find val
    public static double ODO_RADIUS = 0; // radius of odometer wheels TODO: find val

    private final DcMotorEx leftOdoPod, centerOdoPod, rightOdoPod; // we use motorEx bc the ftc sdk doesn't have a dedicated Encoder class, but these will only be used for encoder functionality
    private double prevLeftPos, prevRightPos, prevCenterPos;

    private long prevTime;

    public ThreeWheelOdo(LinearOpMode opMode, Pose startingPose) {
        super(opMode, "Three Wheel Odometry", false, 0.90);

        this.pose = startingPose;

        leftOdoPod = hardwareMap.get(DcMotorEx.class, "left_odo_pod");
        centerOdoPod = hardwareMap.get(DcMotorEx.class, "center_odo_pod");
        rightOdoPod = hardwareMap.get(DcMotorEx.class, "right_odo_pod");

        leftOdoPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerOdoPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdoPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prevTime = System.nanoTime();
    }

    public void update() {
        double leftPos = leftOdoPod.getCurrentPosition() / ODO_ENCODER_RES * ODO_RADIUS;
        double centerPos = centerOdoPod.getCurrentPosition() / ODO_ENCODER_RES * ODO_RADIUS;
        double rightPos = rightOdoPod.getCurrentPosition() / ODO_ENCODER_RES * ODO_RADIUS;

        double leftPosChange = leftPos - prevLeftPos;
        double centerPosChange = centerPos - prevCenterPos;
        double rightPosChange = rightPos - prevRightPos;

        prevLeftPos = leftPos; // forward = positive
        prevCenterPos = centerPos; // forward = no change
        prevRightPos = rightPos; // forward = negative

        double forwardChange = (leftPosChange - rightPosChange) / 2; // checks how much bot has moved forward/backward, turning should cancel out
        double rotationalChange = (leftPosChange + rightPosChange) / 2; // checks how much bot has rotated in linear units, forward/backward movement should cancel out
        double lateralChange = centerPosChange - rotationalChange; // movement of center odoPod excluding rotation

        double hChange = rotationalChange / (TRACK_WIDTH * PI) * 360; // compares the arc (rotationalChange) to total circumfrence of circle in order to find the angle measure (headingChange)

        // convert lateral and forward robot-centric values to field centric coordinates
        pose.h += hChange;
        double heading = toRadians(pose.h);

        // this trig should just rotate the robot's relative coordinate plane (and the points of change on it) to the global position
        // TODO: Mess around with this on the actual robot, heading may need to be inverted
        double xChange = lateralChange * cos(heading) - forwardChange * sin(heading);
        double yChange = lateralChange * sin(heading) + forwardChange * cos(heading);

        pose.x += xChange;
        pose.y += yChange;
    }
}
