package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Indicator;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.util.Global;

@Disabled
@TeleOp(group = Global.OpModeGroup.DO_NOT_BREAK)
public class BasicTeleOp extends OpMode {

    MecDriveBase driveBase;
    Indicator indicator;

    @Override
    public void init() {
        driveBase = new MecDriveBase(this);
        indicator = new Indicator(this);

        telemetry.update();
    }

    @Override
    public void loop() {
        // DRIVEBASE
        driveBase.control(gamepad1);

        telemetry.addData("placeholder", null);
        // UPDATES
        telemetry.update();
    }
}
