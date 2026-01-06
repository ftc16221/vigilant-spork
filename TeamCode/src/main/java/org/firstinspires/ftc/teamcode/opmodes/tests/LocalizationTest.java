package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.util.DashOpMode;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp(group = Global.OpModeGroup.TEST)
public class LocalizationTest extends LinearOpMode implements DashOpMode {

    public void runOpMode() {

        MecDriveBase driveBase = new MecDriveBase(this);
        Navigator navigator = new Navigator(this, Global.lastPose);
        navigator.disableMovement();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driveBase.control(gamepad1);
                navigator.update();

                Pose currentPose = navigator.getCurrentPose();
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
