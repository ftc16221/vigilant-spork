package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public abstract class Localizer extends Subassembly {

    public static double LINEAR_SPEED_TOLERANCE = 3.0;
    public static double ANGULAR_SPEED_TOLERANCE = 2.0;

    protected Pose pose;
    protected Pose velocity = new Pose(0, 0, 0);

    protected final OpMode opMode;
    protected final HardwareMap hardwareMap;
    protected final MultipleTelemetry telemetry;

    public Localizer(OpMode opMode, String name) {
        super(opMode, name);
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = super.getTelemetry();
    }

    public Pose getPose() { return pose; }
    public void setPose(Pose newPose) { pose = newPose; }
    public Pose getVelocity() { return velocity; }
    public double getLinearSpeed() { return Math.hypot(velocity.x, velocity.y); }
    public double getAngularSpeed() { return Math.abs(velocity.h); }
    public boolean isRobotMoving() { return getLinearSpeed() > LINEAR_SPEED_TOLERANCE || getAngularSpeed() > ANGULAR_SPEED_TOLERANCE; }
    public abstract void update();
}