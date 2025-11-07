package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.PoseTracker;
import org.firstinspires.ftc.teamcode.util.DashOpMode;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp(group = Global.OpModeGroup.MAIN)
@Config
public class SemiAutoTeleOp extends LinearOpMode implements DashOpMode {

    public static Pose TARGET_POSE = new Pose(0, 0, 0);
    private boolean autoMovementEnabled = false;

    public void runOpMode() {
        MecDriveBase driveBase = new MecDriveBase(this);
        Intake intake = new Intake(this);
        PoseTracker poseTracker = new PoseTracker(this, Global.lastPose);
        Underglow underglow = new Underglow(this);
        Drawing drawing = new Drawing(poseTracker);

        driveBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        poseTracker.setTargetPose(TARGET_POSE);
        poseTracker.setControllerType(PoseTracker.ControllerType.APPROACH);

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while(opModeIsActive()) {
                // INTAKE
                if (gamepad1.dpadUpWasPressed() || gamepad2.aWasPressed()) {
                    intake.run(Intake.Direction.IN);
                } else if (gamepad1.dpadDownWasPressed() || gamepad2.yWasPressed()) {
                    intake.run(Intake.Direction.OUT);
                } else if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed() || gamepad2.bWasPressed()) {
                    intake.stop();
                }
                // ALL DRIVEBASE MOVEMENT (SEMI-AUTO OR TELEOP)
                if (gamepad1.rightBumperWasPressed()) {
                    autoMovementEnabled = true;
                    poseTracker.enableMovement();
                    underglow.setColor(Underglow.Color.WHITE);
                } else if (!gamepad1.atRest()) {
                    autoMovementEnabled = false;
                    poseTracker.disableMovement();
                    underglow.setColor(Underglow.Color.ALLIANCE);
                }
                if (!autoMovementEnabled) {
                    driveBase.control(gamepad1);
                }
                // UPDATES
                poseTracker.update();
                poseTracker.runTelemetry();
                telemetry.update();
                drawing.update();
            }
        }

    }
}
