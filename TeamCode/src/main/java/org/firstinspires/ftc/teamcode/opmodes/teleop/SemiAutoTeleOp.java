package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Launcher;
import org.firstinspires.ftc.teamcode.subassemblies.Lift;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.subassemblies.Indicator;
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
    public static double CLOSE_LAUNCH_RPM = 3000; // TODO
    public static double CLOSE_LAUNCH_ANGLE = 40; // degrees

    public static Pose FAR_LAUNCH_POSE = new Pose(120, 50, 70); // TODO
    public static double FAR_LAUNCH_RPM = 4000; // Experimented 03/01/2026
    public static double FAR_LAUNCH_ANGLE = 45;

    public static double SLOW_COEFF = 0.2; // 20% speed

    public static int IDLE_COLOR = Color.GREEN;
    public static int GOAL_TRACKING_COLOR = Color.CYAN;
    public static int AUTO_MOVEMENT_COLOR = Color.WHITE;

    MecDriveBase driveBase;
    Spindexer spindexer;
    Launcher launcher;
    Intake intake;
    Lift lift;
    Indicator indicator;
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
        lift = new Lift(this);
        limelightCam = new LimelightCam(this);
        LocalizationManager localizationManager = new LocalizationManager(
                this,
                new PinpointOdo(this, Global.lastPose),
                limelightCam
        );
        navigator = new Navigator(this, localizationManager);
        indicator = new Indicator(this);
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
    public void start() {
        launcher.start();
    }

    @Override
    public void loop() {

        double dt = time - prevTime;
        prevTime = time;

        drawing.prep();
        drawing.drawPoint(GOAL_POSE, "purple");

        if (Global.motif == Global.Motif.UNKNOWN) {
            limelightCam.searchForMotif();
        }

        // ################   GAMEPAD 1   ################
        if (gamepad1.left_bumper || gamepad1.right_stick_button) { // start goal tracking
            goalTrackingEnabled = true;
            navigator.enablePointTracking();
            Indicator.setRobotStatus(GOAL_TRACKING_COLOR);
        } else if (gamepad1.right_stick_x != 0 && goalTrackingEnabled) { // cancel goal tracking
            goalTrackingEnabled = false;
            navigator.disablePointTracking();
            Indicator.setRobotStatus(IDLE_COLOR);
        }

        if (gamepad1.dpadUpWasPressed()) {
            navigator.setUnspecificTargetPose(FAR_LAUNCH_POSE);
            startAutoMovement();
            spindexer.alignAnyForLaunch();
        } else if (gamepad1.dpadDownWasPressed()) {
            navigator.setUnspecificTargetPose(CLOSE_LAUNCH_POSE);
            startAutoMovement();
            spindexer.alignAnyForLaunch();
        }

        if (!gamepad1.atRest() && autoMovementEnabled) {
            stopAutoMovement();
        }

        if (goalTrackingEnabled) {
            double leftX;
            double leftY;
            if (gamepad1.right_trigger > 0.5) {
                leftX = gamepad1.left_stick_x * SLOW_COEFF;
                leftY = -gamepad1.left_stick_y * SLOW_COEFF;
            } else {
                leftX = gamepad1.left_stick_x;
                leftY = -gamepad1.left_stick_y;
            }
            driveBase.moveRobot(leftX, leftY, navigator.getTrackingPower());
        }
        if (!autoMovementEnabled && !goalTrackingEnabled) {
            if (gamepad1.right_trigger > 0.5) {
                driveBase.control(gamepad1, SLOW_COEFF);
            } else {
                driveBase.control(gamepad1);
            }
        }

        // ################   GAMEPAD 2   ################

        // LIFT
        lift.setPower(-gamepad2.left_stick_y);

        if (gamepad2.left_stick_y > 0.2) launcher.setTargetVelocity(0);

        // LAUNCH MODE
        if (gamepad2.right_trigger > 0.2 && !gamepad2RightTriggerWasPressed) {
            launcher.launchAll();
            gamepad2RightTriggerWasPressed = true;
        } else {
            gamepad2RightTriggerWasPressed = false;
        }
        if (gamepad2.rightBumperWasPressed()) {
            launcher.launchAny();
        } else if (gamepad2.yWasPressed()) {
            launcher.launchMotif();
        } else if (gamepad2.aWasPressed()) {
            launcher.launchGreen();
        } else if (gamepad2.xWasPressed()) {
            launcher.launchPurple();
        } else if (gamepad2.bWasPressed()) {
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

        spindexer.manualOffset += -gamepad2.left_stick_x * 2;

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
        indicator.update();
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
        launcher.stop();
        indicator.stop();
    }

    private void startAutoMovement() {
        autoMovementEnabled = true;
        navigator.enableMovement();
        Indicator.setRobotStatus(AUTO_MOVEMENT_COLOR);
    }

    private void stopAutoMovement() {
        autoMovementEnabled = false;
        navigator.disableMovement();
        Indicator.setRobotStatus(IDLE_COLOR);
    }
}
