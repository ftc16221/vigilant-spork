package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Objects;

public class Pose {
    public double x, y , h;

    public Pose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public Pose(SparkFunOTOS.Pose2D sparkFunPose) {
        this.x = sparkFunPose.x;
        this.y = sparkFunPose.y;
        this.h = sparkFunPose.h;
    }

    public Pose(Pose3D pose3D) {
        Position position = pose3D.getPosition();
        this.x = position.x;
        this.y = position.y;
        this.h = pose3D.getOrientation().getYaw(Global.ANGLE_UNIT);
    }

    public SparkFunOTOS.Pose2D toSparkFunPose() {
        return new SparkFunOTOS.Pose2D(x, y, h);
    }

    @NonNull
    public String toString() { return String.format("Pose(x=%.2f, y=%.2f, h=%.2f", x, y, h); }

    public int hashCode() { return Objects.hash(x, y, h); }

    public boolean equals(Pose pose) { return this.hashCode() == pose.hashCode(); }

    public Pose subtract(Pose subtrahend) {
        return new Pose(
                this.x - subtrahend.x,
                this.y - subtrahend.y,
                this.h - subtrahend.h
        );
    }

    public Pose add(Pose addend) {
        return new Pose(
                this.x + addend.x,
                this.y + addend.y,
                this.h + addend.h
        );
    }

    public double distanceFromOrigin() { return Math.hypot(x, y); }

    public double distanceFrom(Pose pose) { return subtract(pose).distanceFromOrigin(); }
}
