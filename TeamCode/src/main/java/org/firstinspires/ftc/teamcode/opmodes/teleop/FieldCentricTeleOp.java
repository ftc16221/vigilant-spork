package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.autonomous.PoseTracker;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = "exploratory")
public class FieldCentricTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        PoseTracker poseTracker = new PoseTracker(this, Global.lastPose);

        poseTracker.disableMovement();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                poseTracker.update();
                if (poseTracker.getCurrentPose() != null) {
                    poseTracker.moveRobotFieldCentric(
                            -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x
                    );
                }
            }
        }
    }
}
