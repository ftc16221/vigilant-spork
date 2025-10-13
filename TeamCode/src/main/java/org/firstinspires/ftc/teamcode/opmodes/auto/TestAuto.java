package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Path;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.PoseTracker;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@Autonomous(group = "test", preselectTeleOp = Global.PRESELECT_TELEOP)
@Config
public class TestAuto extends LinearOpMode {

    public static Pose POSE_1 = new Pose(0, 0, 0);
    public static Pose POSE_2 = new Pose(0, 30, 180);
    public static Pose POSE_3 = new Pose(30, 0, 90);
    public static Pose POSE_4 = new Pose(30, 30, -90);
    public static Pose STARTING_POSE = POSE_2;

    public void runOpMode() {

        PoseTracker poseTracker = new PoseTracker(this, STARTING_POSE);
        Drawing drawing = new Drawing(poseTracker);

        int currentPath = 1;

        Path path1 = new Path(POSE_2, POSE_1, POSE_3);
        Path path2 = new Path(POSE_3, POSE_4, POSE_2);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (currentPath == 1) {
                    path1.execute(poseTracker);
                    if (path1.isComplete()) {
                        currentPath = 2;
                        path2.reinitialize();
                    }
                } else {
                    path2.execute(poseTracker);
                    if (path2.isComplete()) {
                        currentPath = 1;
                        path1.reinitialize();
                    }
                }
                poseTracker.update();
                drawing.update();
            }
        }
    }
}
