package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.subassemblies.Underglow;

@Config
public class Global {
    public static Underglow.Color alliance = Underglow.Color.OFF;
    public static SparkFunOTOS.Pose2D lastPose = null;
}