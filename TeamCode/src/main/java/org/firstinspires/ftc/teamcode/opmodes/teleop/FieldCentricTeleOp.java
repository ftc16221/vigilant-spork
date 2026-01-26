package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.autonomous.LocalizationManager;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.EXPLORATORY)
public class FieldCentricTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {

        LocalizationManager localizationManager = new LocalizationManager(this,
                new PinpointOdo(this, Global.lastPose),
                new LimelightCam(this)
        );
        Navigator navigator = new Navigator(this, localizationManager);

        navigator.disableMovement();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                navigator.update();
                if (navigator.getCurrentPose() != null) {
                    navigator.moveRobotFieldCentric(
                            -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x
                    );
                }
            }
        }
    }
}
