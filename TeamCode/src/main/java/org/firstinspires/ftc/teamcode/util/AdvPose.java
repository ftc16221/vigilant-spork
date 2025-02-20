package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

/**
 * Represents a pose that the robot is targeting. Should not be used to represent actual position or starting position.
 */
public class AdvPose {
    public double x;
    public double y;
    public double h;
    public double tolerance;
    public boolean stopAtEnd;
    public boolean hold = false;

    public AdvPose(double x, double y, double h, double tolerance, boolean stopAtEnd) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.tolerance = tolerance;
        this.stopAtEnd = stopAtEnd;
    }

    public AdvPose(double x, double y, double h, double tolerance, boolean stopAtEnd, boolean hold) {
        this(x, y, h, tolerance, stopAtEnd);
        this.hold = hold;
    }

    public SparkFunOTOS.Pose2D toSparkFunPose() {
        return new SparkFunOTOS.Pose2D(x, y, h);
    }

    public void fromSparkFunPose(SparkFunOTOS.Pose2D pose, double tolerance, boolean stopAtEnd) {
        x = pose.x;
        y = pose.y;
        h = pose.h;
        this.tolerance = tolerance;
        this.stopAtEnd = stopAtEnd;
    }

    public void fromSparkFunPose(SparkFunOTOS.Pose2D pose, double tolerance, boolean stopAtEnd, boolean holdUntil) {
        fromSparkFunPose(pose, tolerance, stopAtEnd);
    }
}
