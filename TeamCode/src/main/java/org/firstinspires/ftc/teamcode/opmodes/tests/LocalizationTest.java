package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.LocalizationManager;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp(group = Global.OpModeGroup.TEST)
public class LocalizationTest extends LinearOpMode{

    public void runOpMode() {

        MecDriveBase driveBase = new MecDriveBase(this);
        LocalizationManager localizationManager = new LocalizationManager(this,
                new PinpointOdo(this, Global.lastPose),
                new LimelightCam(this)
        );
        Navigator navigator = new Navigator(this, localizationManager);
        Drawing drawing = new Drawing(navigator);
        navigator.disableMovement();

        waitForStart();

        if (opModeIsActive()) {
            navigator.start();
            while (opModeIsActive()) {
                driveBase.control(gamepad1);
                navigator.update();
                drawing.prep();
                drawing.update();

                Pose currentPose = navigator.getCurrentPose();
                if (currentPose == null) {
                    telemetry.addLine("currentPose is null");
                } else {
                    telemetry.addData("x", currentPose.x);
                    telemetry.addData("y", currentPose.y);
                    telemetry.addData("h", currentPose.h);
                }
                telemetry.update();
                drawing.send();
            }
        }
    }
}
