package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.LocalizationManager;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@Autonomous(group = Global.OpModeGroup.MAIN, preselectTeleOp = Global.PRESELECT_TELEOP)
@Config
public class PlaceholderAuto extends OpMode {

    public static Pose STARTING_POSE = new Pose(-128, 107, -142);
    public static Pose POSE_1 = new Pose(-32, 33, 38);
    public static Pose POSE_2 = new Pose(-32, -33, 142);
    public static Pose POSE_3 = new Pose(-110, 26, 90);

    private Navigator navigator;
    private Watchdog watchdog;
    private PinpointOdo pinpointOdo;
    private LimelightCam limelightCam;

    private State state = State.NOT_STARTED;

    @Override
    public void init() {


        Pose trueStartingPose;
        if (Global.alliance == Global.Alliance.BLUE) {
            trueStartingPose = STARTING_POSE.mirror();
        } else {
            trueStartingPose = STARTING_POSE;
        }
        pinpointOdo = new PinpointOdo(this, trueStartingPose);
        limelightCam = new LimelightCam(this);
        LocalizationManager localizationManager = new LocalizationManager(this, pinpointOdo, limelightCam);
        navigator = new Navigator(this, localizationManager);
        watchdog = new Watchdog(this);
    }

    @Override
    public void loop() {



        switch (state) {
            case NOT_STARTED:

                break;
            case MOVING_TO_ACTION:

                break;
            case ACTIONing:

                break;
            case MOVING_TO_END:

        }

        navigator.update();
        watchdog.update();

        telemetry.addData("state", state);
        telemetry.addData("navigator isAtTarget", navigator.isAtTarget());
    }

    private enum State {
        NOT_STARTED, MOVING_TO_ACTION, ACTIONing, MOVING_TO_END
    }
}
