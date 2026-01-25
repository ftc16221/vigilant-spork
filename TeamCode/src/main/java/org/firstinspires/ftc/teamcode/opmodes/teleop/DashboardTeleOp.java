package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.GenericCam;
import org.firstinspires.ftc.teamcode.util.Global;

@Disabled
@TeleOp(group = Global.OpModeGroup.EXPLORATORY)
public class DashboardTeleOp extends OpMode {

    MecDriveBase driveBase;
    GenericCam cam;

    @Override
    public void init() {
        driveBase = new MecDriveBase(this);
        cam = new GenericCam(this);

        FtcDashboard.getInstance().startCameraStream(cam.getDash(), 0.0);
    }

    @Override
    public void loop() {
        driveBase.control(gamepad1);
    }
}
