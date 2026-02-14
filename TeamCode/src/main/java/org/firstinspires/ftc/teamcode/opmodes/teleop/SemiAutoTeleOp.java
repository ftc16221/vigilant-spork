package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Launcher;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.LocalizationManager;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.DashOpMode;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp(group = Global.OpModeGroup.MAIN)
@Config
public class SemiAutoTeleOp extends OpMode implements DashOpMode {

    public static Pose GOAL_POSE = new Pose(-180, 180, 0);

    public static Pose CLOSE_LAUNCH_POSE = new Pose(-51, 30, 50.6); // TODO
    public static double CLOSE_LAUNCH_RPM = 2400; // TODO
    public static double CLOSE_LAUNCH_ANGLE = 40; // degrees

    public static Pose FAR_LAUNCH_POSE = new Pose(120, 50, 70); // TODO
    public static double FAR_LAUNCH_RPM = 3200; // TODO
    public static double FAR_LAUNCH_ANGLE = 45;


    public static Underglow.Color IDLE_COLOR = Underglow.Color.ALLIANCE;
    public static Underglow.Color GOAL_TRACKING_COLOR = Underglow.Color.GREEN;
    public static Underglow.Color AUTO_MOVEMENT_COLOR = Underglow.Color.WHITE;

    MecDriveBase driveBase;
    Spindexer spindexer;
    Launcher launcher;
    Intake intake;
    Underglow underglow;
    Navigator navigator;
    LimelightCam limelightCam;
    Drawing drawing;
    Watchdog watchdog;

    private boolean autoMovementEnabled = false;
    private boolean goalTrackingEnabled = false;

    private boolean gamepad2RightTriggerWasPressed = false;
    private double prevTime = 0;

    @Override
    public void init() {
        driveBase = new MecDriveBase(this);
        intake = new Intake(this);
        spindexer = new Spindexer(this, intake);
        launcher = new Launcher(this, spindexer);
        limelightCam = new LimelightCam(this);
        LocalizationManager localizationManager = new LocalizationManager(
                this,
                new PinpointOdo(this, Global.lastPose),
                limelightCam
        );
        navigator = new Navigator(this, localizationManager);
        underglow = new Underglow(this);
        drawing = new Drawing(navigator);
        watchdog = new Watchdog(this);

        driveBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        navigator.setTargetPose(FAR_LAUNCH_POSE);
        navigator.setUnspecificTrackingPoint(GOAL_POSE);
    }

    @Override
    public void init_loop() {
        limelightCam.searchForMotif();
    }

    @Override
    public void loop() {
        double dt = time - prevTime;
        prevTime = time;

        drawing.prep();
        drawing.drawPoint(GOAL_POSE, "purple");

        // ################   GAMEPAD 1   ################
        /*else if (gamepad1.left_bumper || gamepad1.right_stick_button) { // start goal tracking
            goalTrackingEnabled = true;
            navigator.enablePointTracking();
            underglow.setColor(GOAL_TRACKING_COLOR);
        } else if (gamepad1.right_stick_x != 0 && goalTrackingEnabled) { // cancel goal tracking
            goalTrackingEnabled = false;
            navigator.disablePointTracking();
            underglow.setColor(IDLE_COLOR);
        } TODO commented because there is some PID feedback loop happening somewhere*/

        if (gamepad1.dpadUpWasPressed()) {
            navigator.setUnspecificTargetPose(FAR_LAUNCH_POSE);
            startAutoMovement();
            spindexer.alignAnyForLaunch();
        } else if (gamepad1.dpadDownWasPressed()) {
            navigator.setUnspecificTargetPose(CLOSE_LAUNCH_POSE);
            startAutoMovement();
            spindexer.alignAnyForLaunch();
        }

        if (!gamepad1.atRest()) {
            stopAutoMovement();
        }

        if (goalTrackingEnabled) {
            driveBase.moveRobot(gamepad1.left_stick_x, -gamepad1.left_stick_y, navigator.getTrackingPower());
        }
        if (!autoMovementEnabled && !goalTrackingEnabled) {
            driveBase.control(gamepad1);
        }

        // ################   GAMEPAD 2   ################

        // LAUNCH MODE
        if (gamepad2.right_trigger > 0.2 && !gamepad2RightTriggerWasPressed) {
            launcher.launchAll();
            gamepad2RightTriggerWasPressed = true;
        } else {
            gamepad2RightTriggerWasPressed = false;
        }
        if (gamepad2.rightBumperWasPressed()) {
            launcher.launchAny();
        } else if (gamepad2.yWasPressed() || gamepad2.triangleWasPressed()) {
            launcher.launchMotif();
        } else if (gamepad2.aWasPressed() || gamepad2.crossWasPressed()) {
            launcher.launchGreen();
        } else if (gamepad2.xWasPressed() || gamepad2.squareWasPressed()) {
            launcher.launchPurple();
        } else if (gamepad2.bWasPressed() || gamepad2.circleWasPressed()) {
            launcher.cancelLaunches();
        } else if (gamepad2.rightStickButtonWasPressed()) {
            spindexer.emptyActiveSlot();
            launcher.cancelLaunches();
        } else if (gamepad2.leftStickButtonWasPressed()) {
            // if the kicker is stuck inside the spindexer, we need to release the pressure
            // and retract the kicker.
            // after that, the driver should be able to re-do whatever action needs to be done.
            spindexer.stop();
            launcher.unkick();
        }

        if (gamepad2.dpadUpWasPressed()) {
            launcher.setTargetVelocity(FAR_LAUNCH_RPM);
            launcher.setHoodAngle(FAR_LAUNCH_ANGLE);
        } else if (gamepad2.dpadDownWasPressed()) {
            launcher.setTargetVelocity(CLOSE_LAUNCH_RPM);
            launcher.setHoodAngle(CLOSE_LAUNCH_ANGLE);
        } else if (gamepad2.dpadRightWasPressed()) {
            launcher.setTargetVelocity(0);
            launcher.setHoodAngle(0);
        }

        // INTAKE MODE
        if (gamepad2.dpadLeftWasPressed()) {
            spindexer.alignForIntake();
        }

        telemetry.addData("loop time", dt);

        // UPDATES
        navigator.update();
        launcher.update();
        spindexer.update();
        watchdog.update();
        drawing.update();
        drawing.send();
        navigator.runTelemetry();
        spindexer.runTelemetry();
    }

    @Override
    public void stop() {
        navigator.stop();
        watchdog.stop();
        spindexer.stop();
    }

    private void startAutoMovement() {
        autoMovementEnabled = true;
        navigator.enableMovement();
        underglow.setColor(AUTO_MOVEMENT_COLOR);
    }

    private void stopAutoMovement() {
        autoMovementEnabled = false;
        navigator.disableMovement();
        underglow.setColor(IDLE_COLOR);
    }
}
