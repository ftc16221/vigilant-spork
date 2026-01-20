package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.MAIN)
public class DriveTeleOp extends OpMode {

    MecDriveBase driveBase;

    @Override
    public void init() {
        driveBase = new MecDriveBase(this);
    }

    @Override
    public void loop() {
        telemetry.addData("G1 Left X", gamepad1.left_stick_x);
        telemetry.addData("G1 Left Y", gamepad1.left_stick_y);
        telemetry.addData("G1 Right X", gamepad1.right_stick_x);
        telemetry.addData("G1 Right Y", gamepad1.right_stick_y);

        driveBase.control(gamepad1);
    }

}
