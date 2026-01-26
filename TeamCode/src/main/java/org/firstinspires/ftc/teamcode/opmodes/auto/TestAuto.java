package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subassemblies.autonomous.LocalizationManager;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Path;
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

        LocalizationManager localizationManager = new LocalizationManager(
                this,
                STARTING_POSE,
                new PinpointOdo(this, STARTING_POSE),
                new LimelightCam(this)
        );
        Navigator navigator = new Navigator(this, localizationManager);
        Drawing drawing = new Drawing(navigator);

        int currentPath = 1;

        Path path1 = new Path(POSE_2, POSE_1, POSE_3);
        Path path2 = new Path(POSE_3, POSE_4, POSE_2);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (currentPath == 1) {
                    path1.execute(navigator);
                    if (path1.isComplete()) {
                        currentPath = 2;
                        path2.reinitialize();
                    }
                } else {
                    path2.execute(navigator);
                    if (path2.isComplete()) {
                        currentPath = 1;
                        path1.reinitialize();
                    }
                }
                navigator.update();
                drawing.update();
            }
        }
    }
}
