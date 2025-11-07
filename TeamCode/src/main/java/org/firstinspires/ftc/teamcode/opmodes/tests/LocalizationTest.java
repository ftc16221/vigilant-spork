package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.PoseTracker;
import org.firstinspires.ftc.teamcode.util.DashOpMode;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp(group = "tests")
public class LocalizationTest extends LinearOpMode implements DashOpMode {

    public void runOpMode() {

        MecDriveBase driveBase = new MecDriveBase(this);
        PoseTracker poseTracker = new PoseTracker(this, Global.lastPose);
        poseTracker.disableMovement();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driveBase.control(gamepad1);
                poseTracker.update();

                Pose currentPose = poseTracker.getCurrentPose();
                if (currentPose == null) {
                    telemetry.addLine("currentPose is null");
                } else {
                    telemetry.addData("x", currentPose.x);
                    telemetry.addData("y", currentPose.y);
                    telemetry.addData("h", currentPose.h);
                }
            }
        }
    }
}
