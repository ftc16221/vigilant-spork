package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp(group = Global.OpModeGroup.TEST)
public class PinpointTest extends LinearOpMode{

    MecDriveBase driveBase;
    PinpointOdo pinpointOdo;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        driveBase = new MecDriveBase(this);
        pinpointOdo = new PinpointOdo(this, Global.lastPose);

        driveBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("This opmode is used to find robot positions for autonomous. It is not intended " +
                "for driving the robot, but may be useful for debugging. Currently, the robot's position is purely " +
                "based off of the set starting position. If the starting position is not 0, 0, 0 (center of the field) " +
                "then you will need to change it via FTC Dashboard. I recommend 2 tiles left of the origin (-48, 0, 0), " +
                "because it is accessible and a good anchor point.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                ElapsedTime updateTimer = new ElapsedTime();
                pinpointOdo.update();
                telemetry.addData("update time", updateTimer.milliseconds());
                ElapsedTime readTimer = new ElapsedTime();
                Pose currentPose = pinpointOdo.getPose();
                telemetry.addData("read time", readTimer.milliseconds());
                if (currentPose != null) {
                    telemetry.addData("x", currentPose.x);
                    telemetry.addData("y", currentPose.y);
                    telemetry.addData("h", currentPose.h);
                } else {
                    telemetry.addLine("currentPose is NULL");
                }
                telemetry.update();

                driveBase.control(gamepad1);
                if (gamepad1.bWasPressed()) {
                    pinpointOdo.recalibrate();
                } else if (gamepad1.yWasPressed()) {
                    pinpointOdo.setPose(new Pose(0, 0, 0));
                }
            }
        }
    }
}
