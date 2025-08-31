package org.firstinspires.ftc.teamcode.subassemblies;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Subassembly;

public class Odometry extends Subassembly {

    public static double TRACK_WIDTH = 0; // distance between the middle of the wheels TODO: find val
    public static double ODO_ENCODER_RES = 0; // ticks per rev TODO: find val
    public static double ODO_RADIUS = 0; // radius of odometer wheels TODO: find val
    public static double ROBOT_MOVEMENT_SPEED_THRESHOLD = 3.0;

    private Pose pose;
    private Pose velocity = new Pose(0,0,0);

    private final LinearOpMode opMode;
    private final HardwareMap hardwareMap;

    private final DcMotorEx leftOdoPod, centerOdoPod, rightOdoPod; // we use motorEx bc the ftc sdk doesn't have a dedicated Encoder class, but these will only be used for encoder functionality
    private double prevLeftPos, prevRightPos, prevCenterPos;

    private long prevTime = 0;
    
    public Odometry(LinearOpMode opMode, Pose startingPose) {
        super(opMode, "Odometry");
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;

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

        long currentTime = System.nanoTime();
        double dt = (currentTime - prevTime) / 1e9; // dt for delta time (change in time); in sec

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

        velocity.x = xChange / dt;
        velocity.y = yChange / dt;
        velocity.h = hChange / dt;

        prevTime = currentTime;
    }

    public Pose getPose() { return pose; }
    public void setPose(Pose pose) { this.pose = pose; }

    public Pose getVelocity() { return velocity; }
    public double getSpeed() { return Math.hypot(velocity.x, velocity.y); }
    public boolean isRobotMoving() { return getSpeed() > ROBOT_MOVEMENT_SPEED_THRESHOLD; }

    public void runTelemetry() {
        Telemetry telemetry = getTelemetry();
        telemetry.addLine("Odometry Data");
        telemetry.addData("position data","x=%.1f, y=%.1f, h=%.1f", pose.x, pose.y, pose.h);
        telemetry.addData("velocity data","x=%.1f, y=%.1f, h=%.1f", velocity.x, velocity.y, velocity.h);
    }
}