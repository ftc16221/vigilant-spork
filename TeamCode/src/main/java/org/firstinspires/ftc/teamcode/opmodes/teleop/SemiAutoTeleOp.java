package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.PoseTracker;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp
@Config
public class SemiAutoTeleOp extends LinearOpMode {

    public static Pose TARGET_POSE = new Pose(0, 0, 0);
    private boolean autoMovementEnabled = false;

    public void runOpMode() {
        MecDriveBase driveBase = new MecDriveBase(this);
        PoseTracker poseTracker = new PoseTracker(this, Global.lastPose);
        Underglow underglow = new Underglow(this);

        poseTracker.setTargetPose(TARGET_POSE);
        poseTracker.setControllerType(PoseTracker.ControllerType.APPROACH);

        waitForStart();

        if (opModeIsActive()) {
            while(opModeIsActive()) {
                if (gamepad1.right_bumper) {
                    autoMovementEnabled = true;
                    poseTracker.enableMovement();
                    underglow.setColor(Underglow.Color.WHITE);
                } else if (!gamepad1.atRest()) {
                    autoMovementEnabled = false;
                    poseTracker.disableMovement();
                    underglow.setColor(Underglow.Color.ALLIANCE);
                }
                if (!autoMovementEnabled) {
                    driveBase.moveRobot(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
                }
                poseTracker.update();
            }
        }
    }
}
