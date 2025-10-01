package org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Subassembly;

@Config
public abstract class Localizer extends Subassembly {

    public static double ROBOT_MOVEMENT_SPEED_THRESHOLD = 3.0;

    protected Pose pose;
    protected Pose velocity = new Pose(0,0,0);

    protected final LinearOpMode opMode;
    protected final HardwareMap hardwareMap;
    protected final Telemetry telemetry;

    public Localizer(LinearOpMode opMode, String name) {
        super(opMode, name);
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public Pose getPose() { return pose; }
    public void setPose(Pose newPose) { pose = newPose; }
    public Pose getVelocity() { return velocity; }
    public double getSpeed() { return Math.hypot(velocity.x, velocity.y); }
    public boolean isRobotMoving() { return getSpeed() > ROBOT_MOVEMENT_SPEED_THRESHOLD; }
    public abstract void update();
}