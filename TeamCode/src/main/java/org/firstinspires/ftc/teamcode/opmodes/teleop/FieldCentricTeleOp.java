package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Follower;

@TeleOp(group = "exploratory")
public class FieldCentricTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new SparkFunOTOS.Pose2D(0, 0, 180));

        follower.disable();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                follower.moveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
        }
    }
}
