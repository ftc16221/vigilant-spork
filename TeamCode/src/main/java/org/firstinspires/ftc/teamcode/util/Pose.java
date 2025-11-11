package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Objects;

public class Pose {
    public double x, y, h;

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

    public Pose(Pose3D pose3D, DistanceUnit distanceUnit) {
        Position position = pose3D.getPosition();
        this.x = distanceUnit.toCm(position.x);
        this.y = distanceUnit.toCm(position.y);
        this.h = pose3D.getOrientation().getYaw(Global.ANGLE_UNIT);
    }

    // The absolute stupidest of conversion, to try and deal with the fact apriltag metadata positions use different units than literally everything else in the SDK and even other aprilTag stuff :)
    public Pose(VectorF positionVectorF, Quaternion orientationQuaternion) {
        this.x = Global.DISTANCE_UNIT.fromInches(positionVectorF.get(0));
        this.y = Global.DISTANCE_UNIT.fromInches(positionVectorF.get(1));
        Orientation orientation = Orientation.getOrientation(
                orientationQuaternion.toMatrix(),
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                Global.ANGLE_UNIT
        );
        this.h = orientation.thirdAngle; // third angle is z axis, which is yaw/heading
    }

    public SparkFunOTOS.Pose2D toSparkFunPose() {
        return new SparkFunOTOS.Pose2D(x, y, h);
    }

    @NonNull
    @Override
    public String toString() { return String.format("Pose(x=%.2f, y=%.2f, h=%.2f", x, y, h); }

    @Override
    public int hashCode() { return Objects.hash(x, y, h); }

    public boolean equals(Pose pose) { return this.hashCode() == pose.hashCode(); }

    public Pose add(Pose addend) {
        return new Pose(
                this.x + addend.x,
                this.y + addend.y,
                this.h + addend.h
        );
    }

    public Pose subtract(Pose subtrahend) {
        return new Pose(
                this.x - subtrahend.x,
                this.y - subtrahend.y,
                this.h - subtrahend.h
        );
    }

    public Pose multiplyBy(double factor) {
        return new Pose(
                this.x * factor,
                this.y * factor,
                this.h * factor
        );
    }

    public Pose divideBy(double divisor) {
        return new Pose(
                this.x / divisor,
                this.y / divisor,
                this.h / divisor
        );
    }

    public double getDistanceFromOrigin() { return Math.hypot(x, y); }

    public double getDistanceFromPose(Pose pose) { return subtract(pose).getDistanceFromOrigin(); }

    public Pose rotate(double angle) {
        double angleInRads = Global.ANGLE_UNIT.toRadians(angle);
        double rotatedX = x * Math.cos(angleInRads) - y * Math.sin(angleInRads);
        double rotatedY = x * Math.sin(angleInRads) + y * Math.cos(angleInRads);
        double rotatedH = AngleUnit.normalizeDegrees(h + angle);
        return new Pose(rotatedX, rotatedY, rotatedH);
    }

    public Pose invert() {
        return rotate(180);
    }

    public Pose mirror() {
        return new Pose(x, -y, h);
    }

    public Pose toFieldCentric(double currentHeading) { return rotate(currentHeading); }

    public Pose toRobotCentric(double currentHeading) { return rotate(-currentHeading); }
}
