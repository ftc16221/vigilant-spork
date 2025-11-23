package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.PoseTracker;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.DashOpMode;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp(group = Global.OpModeGroup.MAIN)
@Config
@Configurable
public class SemiAutoTeleOp extends OpMode implements DashOpMode {

    public static Pose TARGET_POSE = new Pose(0, 0, 0);
    public static Pose TRACKING_POINT = new Pose(0, 0, 0);

    public static Underglow.Color IDLE_COLOR = Underglow.Color.ALLIANCE;
    public static Underglow.Color GOAL_TRACKING_COLOR = Underglow.Color.GREEN;
    public static Underglow.Color AUTO_MOVEMENT_COLOR = Underglow.Color.WHITE;

    MecDriveBase driveBase;
    Intake intake;
    Underglow underglow;
    PoseTracker poseTracker;
    LimelightCam limelightCam;
    PinpointOdo pinpointOdo;
    Drawing drawing;

    private boolean autoMovementEnabled = false;
    private boolean goalTrackingEnabled = false;

    @Override
    public void init() {
        driveBase = new MecDriveBase(this);
        intake = new Intake(this);
        limelightCam = new LimelightCam(this);
        pinpointOdo = new PinpointOdo(this, Global.lastPose);
        poseTracker = new PoseTracker(this, Global.lastPose);
        underglow = new Underglow(this);
        drawing = new Drawing(poseTracker);

        driveBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        poseTracker.setTargetPose(TARGET_POSE);
        poseTracker.setTrackingPoint(TRACKING_POINT);
        poseTracker.setControllerType(PoseTracker.ControllerType.APPROACH);
    }

    @Override
    public void loop() {

        // INTAKE
        if (gamepad1.dpad_up || gamepad2.a) {
            intake.run(Intake.Direction.IN);
        } else if (gamepad1.dpad_down || gamepad2.y) {
            intake.run(Intake.Direction.OUT);
        } else if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad2.b) {
            intake.stop();
        }

        // ALL DRIVEBASE MOVEMENT (SEMI-AUTO OR TELEOP)
        if (gamepad1.right_bumper) { // start auto movement
            autoMovementEnabled = true;
            poseTracker.enableMovement();
            underglow.setColor(AUTO_MOVEMENT_COLOR);
        } else if (!gamepad1.atRest() && autoMovementEnabled) { // cancel auto movement
            autoMovementEnabled = false;
            poseTracker.disableMovement();
            underglow.setColor(IDLE_COLOR);
        } else if (gamepad1.left_bumper || gamepad1.right_stick_button) { // start goal tracking
            goalTrackingEnabled = true;
            poseTracker.enablePointTracking();
            underglow.setColor(GOAL_TRACKING_COLOR);
        } else if (gamepad1.right_stick_x != 0 && goalTrackingEnabled) { // cancel goal tracking
            goalTrackingEnabled = false;
            poseTracker.disablePointTracking();
            underglow.setColor(IDLE_COLOR);
        }

        if (goalTrackingEnabled) {
            driveBase.moveRobot(gamepad1.left_stick_x, -gamepad1.left_stick_y, poseTracker.getTrackingPower());
        }
        if (!autoMovementEnabled && !goalTrackingEnabled) {
            driveBase.control(gamepad1);
        }

        // UPDATES
        poseTracker.update();
        poseTracker.runTelemetry();
        telemetry.update();
        drawing.update();
    }
}
