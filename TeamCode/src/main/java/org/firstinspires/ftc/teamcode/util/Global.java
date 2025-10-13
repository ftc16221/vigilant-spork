package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config
public class Global {
    public static Global.Alliance alliance = null;
    public static Pose lastPose = new Pose(0, 0, 0);
    public static final String PRESELECT_TELEOP = "SimpleTeleOp";
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.CM;
    public static AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    public static UnnormalizedAngleUnit UNNORMALIZED_ANGLE_UNIT = UnnormalizedAngleUnit.DEGREES; // should always be the same as ANGLE_UNIT

    public enum Alliance {
        RED,
        BLUE
    }
}