package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.EXPLORATORY)
public class FieldCentricTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Navigator navigator = new Navigator(this, Global.lastPose);

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
