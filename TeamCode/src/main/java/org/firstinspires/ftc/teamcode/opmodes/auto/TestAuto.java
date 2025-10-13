package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Path;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.PoseTracker;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@Autonomous(group = "test", preselectTeleOp = Global.PRESELECT_TELEOP)
@Config
public class TestAuto extends LinearOpMode {

    public static Pose STARTING_POSE = new Pose(0, 0, 0);
    public static Path PATH_1 = new Path(
            new Pose(0, 0, 0),
            new Pose(0, 30, 180)
    );
    public static Path PATH_2 = new Path(
            new Pose(0, 30, 180),
            new Pose (0, 0, 0)
    );

    public void runOpMode() {

        PoseTracker poseTracker = new PoseTracker(this, STARTING_POSE);
        int currentPath = 1;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (currentPath == 1) {
                    PATH_1.execute(poseTracker);
                    if (PATH_1.isComplete()) {
                        currentPath = 2;
                        PATH_2.reinitialize();
                    }
                } else {
                    PATH_2.execute(poseTracker);
                    if (PATH_2.isComplete()) {
                        currentPath = 1;
                        PATH_1.reinitialize();
                    }
                }
                poseTracker.update();
            }
        }
    }
}
