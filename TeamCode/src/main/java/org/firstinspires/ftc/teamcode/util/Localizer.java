package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

//@Config
public abstract class Localizer extends Subassembly {

    public final boolean isAbsolute;
    public final double accuracy;

    protected Pose pose;

    protected final OpMode opMode;
    protected final HardwareMap hardwareMap;

    public Localizer(OpMode opMode, String name, boolean isAbsolute, double accuracy) {
        super(opMode, name);
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.isAbsolute = isAbsolute;
        this.accuracy = accuracy;
    }

    public Pose getPose() { return pose; }
    public void setPose(Pose newPose) { pose = newPose; }
    public abstract void update();
}