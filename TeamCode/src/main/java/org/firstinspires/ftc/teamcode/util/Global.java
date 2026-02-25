package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config
public class Global {
    public static Global.Alliance alliance = null;
    public static Motif motif = Motif.GPP;
    public static Pose lastPose = new Pose(0, 0, 0);

    public static boolean ENABLE_TUNING_MODE = false; // when enabled this will update PIDF coefficients in real time

    public static final String PRESELECT_TELEOP = "SimpleTeleOp";
    public static final DistanceUnit DISTANCE_UNIT = DistanceUnit.CM;
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    public static final UnnormalizedAngleUnit UNNORMALIZED_ANGLE_UNIT = UnnormalizedAngleUnit.DEGREES; // should always be the same as ANGLE_UNIT

    public static void reset() {
        alliance = null;
        motif = null;
        lastPose = new Pose(0, 0, 0);
    }

    public enum Alliance {
        RED,
        BLUE
    }

    public enum Motif {
        GPP, PGP, PPG, UNKNOWN
    }

    public static class OpModeGroup {
        public static final String SETUP = "1 - Setup";
        public static final String MAIN = "2 - Main";
        public static final String DO_NOT_BREAK = "3 - Do not break";
        public static final String TEST = "4 - Test";
        public static final String TUNER = "5 - Tuner";
        public static final String EXPLORATORY = "6 - Exploratory";
    }
}