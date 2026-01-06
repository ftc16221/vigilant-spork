package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Launcher;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@Autonomous(group = Global.OpModeGroup.MAIN, preselectTeleOp = Global.PRESELECT_TELEOP)
@Config
public class ThreeBallAuto extends OpMode {

    public static Pose STARTING_POSE = new Pose(0, 0, 0);
    public static Pose LAUNCH_POSE = new Pose(0, 0, 0);
    public static Pose END_POSE = new Pose(0, 0, 0);

    public static double LAUNCH_SPEED = 3400.0; // RPM
    public static double LAUNCH_ANGLE = 45.0; // degrees

//    private Navigator navigator;
    private Intake intake;
    private Spindexer spindexer;
    private Launcher launcher;
    private Watchdog watchdog;

    private State state = State.NOT_STARTED;

    @Override
    public void init() {
//        navigator = new Navigator(this, STARTING_POSE);
        intake = new Intake(this);
        spindexer = new Spindexer(this, intake);
        launcher = new Launcher(this, spindexer);
        watchdog = new Watchdog(this, spindexer);
    }

    @Override
    public void loop() {
        switch (state) {
            case NOT_STARTED:
//                navigator.setTargetPose(LAUNCH_POSE);
                launcher.setTargetVelocity(LAUNCH_SPEED);
                launcher.setHoodAngle(LAUNCH_ANGLE);
                state = State.MOVING_TO_LAUNCH;
                break;
            case MOVING_TO_LAUNCH:
                if (/*navigator.isAtTarget()*/true) {
                    launcher.launchMotif();
                    state = State.LAUNCHING;
                }
                break;
            case LAUNCHING:
                if (spindexer.isEmpty()) {
//                    navigator.setTargetPose(END_POSE);
                    state = State.MOVING_TO_END;
                }
                break;
            case MOVING_TO_END:
                if (/*navigator.isAtTarget()*/true) {
//                    navigator.stop();
                    intake.stop();
                    spindexer.stop();
                    requestOpModeStop();
                }
        }

        spindexer.update();
        launcher.update();
        watchdog.update();

        telemetry.addData("state", state);
//        telemetry.addData("navigator isAtTarget", navigator.isAtTarget());
        telemetry.addData("spindexer isBusy", spindexer.isBusy());
        telemetry.addData("launcher isReady", launcher.isReady());
        telemetry.addData("launcher state", launcher.getState());
    }

    private enum State {
        NOT_STARTED, MOVING_TO_LAUNCH, LAUNCHING, MOVING_TO_END
    }
}
