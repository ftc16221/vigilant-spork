package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subassemblies.Indicator;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.LocalizationManager;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp(group = Global.OpModeGroup.MAIN)
@Config
public class SemiAutoTeleOp extends OpMode{

    // TODO: find purpose / delete for next season
    public static Pose PLACEHOLDER_POSE_1 = new Pose(-180, 180, 0);
    public static Pose PLACEHOLDER_POSE_2 = new Pose(-51, 30, 50.6);
    public static Pose PLACEHOLDER_POSE_3 = new Pose(120, 50, 70);

    public static double SLOW_COEFF = 0.2; // 20% speed

    // Colors for the indicator
    public static int IDLE_COLOR = Color.GREEN;
    public static int GOAL_TRACKING_COLOR = Color.CYAN;
    public static int AUTO_MOVEMENT_COLOR = Color.WHITE;

    MecDriveBase driveBase;
    Indicator indicator;
    Navigator navigator;
    LimelightCam limelightCam;
    Drawing drawing;
    Watchdog watchdog;

    private boolean autoMovementEnabled = false;
    private boolean goalTrackingEnabled = false;

    private double prevTime = 0;

    @Override
    public void init() {
        driveBase = new MecDriveBase(this);
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

        navigator.setTargetPose(PLACEHOLDER_POSE_3);
        navigator.setUnspecificTrackingPoint(PLACEHOLDER_POSE_1);
    }

    @Override
    public void loop() {

        double dt = time - prevTime;
        prevTime = time;

        drawing.prep();
        drawing.drawPoint(PLACEHOLDER_POSE_1, "purple");

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
            navigator.setUnspecificTargetPose(PLACEHOLDER_POSE_3);
            startAutoMovement();

        } else if (gamepad1.dpadDownWasPressed()) {
            navigator.setUnspecificTargetPose(PLACEHOLDER_POSE_2);
            startAutoMovement();

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

        telemetry.addData("loop time", dt);

        // UPDATES
        navigator.update();
        watchdog.update();
        indicator.update();
        drawing.update();
        drawing.send();
        navigator.runTelemetry();
    }

    @Override
    public void stop() {
        navigator.stop();
        watchdog.stop();
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
