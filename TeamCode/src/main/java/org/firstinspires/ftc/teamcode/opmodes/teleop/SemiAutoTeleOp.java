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

    public static Pose TARGET_POSE = new Pose(0, 0, 0);
    public static Pose TRACKING_POINT = new Pose(0, 0, 0);

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
    PinpointOdo pinpointOdo;
    Drawing drawing;
    Watchdog watchdog;

    private boolean autoMovementEnabled = false;
    private boolean goalTrackingEnabled = false;

    private boolean gamepad2RightTriggerWasPressed = false;

    @Override
    public void init() {
        driveBase = new MecDriveBase(this);
        intake = new Intake(this);
        spindexer = new Spindexer(this, intake);
        launcher = new Launcher(this, spindexer);
        limelightCam = new LimelightCam(this);
        pinpointOdo = new PinpointOdo(this, Global.lastPose);
        navigator = new Navigator(this, Global.lastPose);
        underglow = new Underglow(this);
        drawing = new Drawing(navigator);
        watchdog = new Watchdog(this);

        driveBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        navigator.setTargetPose(TARGET_POSE);
        navigator.setTrackingPoint(TRACKING_POINT);
        navigator.setControllerType(Navigator.ControllerType.APPROACH);
    }

    @Override
    public void loop() {

        // ################   GAMEPAD 1   ################
        if (gamepad1.right_bumper) { // start auto movement
            autoMovementEnabled = true;
            navigator.enableMovement();
            underglow.setColor(AUTO_MOVEMENT_COLOR);
        } else if (!gamepad1.atRest() && autoMovementEnabled) { // cancel auto movement
            autoMovementEnabled = false;
            navigator.disableMovement();
            underglow.setColor(IDLE_COLOR);
        } else if (gamepad1.left_bumper || gamepad1.right_stick_button) { // start goal tracking
            goalTrackingEnabled = true;
            navigator.enablePointTracking();
            underglow.setColor(GOAL_TRACKING_COLOR);
        } else if (gamepad1.right_stick_x != 0 && goalTrackingEnabled) { // cancel goal tracking
            goalTrackingEnabled = false;
            navigator.disablePointTracking();
            underglow.setColor(IDLE_COLOR);
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
        }

        // INTAKE MODE
        if (gamepad2.dpadUpWasPressed()) {
            spindexer.alignForIntake();
        }

        // UPDATES
        navigator.update();
        launcher.update();
        spindexer.update();
        navigator.runTelemetry();
        telemetry.update();
        drawing.update();
    }

    @Override
    public void stop() {
        poseTracker.stop();
        watchdog.stop();
    }
}
