package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subassemblies.Indicator;
import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Launcher;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.LocalizationManager;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.TEST)
public class LoopTimeTest extends OpMode {

    MecDriveBase driveBase;
    Spindexer spindexer;
    Launcher launcher;
    Intake intake;
    Indicator indicator;
    Navigator navigator;
    LimelightCam limelightCam;
    Drawing drawing;
    Watchdog watchdog;

    double lastTime = 0;

    @Override public void init() {
        driveBase = new MecDriveBase(this);
        intake = new Intake(this);
        spindexer = new Spindexer(this, intake);
        launcher = new Launcher(this, spindexer);
        limelightCam = new LimelightCam(this);
        LocalizationManager localizationManager = new LocalizationManager(
                this,
                new PinpointOdo(this, Global.lastPose),
                limelightCam
        );
        navigator = new Navigator(this, localizationManager);
        indicator = new Indicator(this);
        drawing = new Drawing(navigator);
        watchdog = new Watchdog(this);

        driveBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override public void loop() {
        double runtime = getRuntime();
        telemetry.addData("Total", runtime);
        telemetry.addData("OpMode Processes", "%.3f", timeSinceLastCall());
        driveBase.control(gamepad1);
        telemetry.addData("DriveBase", "%.3f", timeSinceLastCall());
        navigator.update();
        telemetry.addData("Navigator", "%.3f", timeSinceLastCall());
        launcher.update();
        telemetry.addData("Launcher", "%.3f", timeSinceLastCall());
        spindexer.update();
        telemetry.addData("Spindexer", "%.3f", timeSinceLastCall());
        watchdog.update();
        telemetry.addData("Watchdog", "%.3f", timeSinceLastCall());
        indicator.update();
        telemetry.addData("Indicator", "%.3f", timeSinceLastCall());

//        drawing.update();
//        drawing.send();
//        telemetry.addData("Drawing", timeSinceLastCall());
    }

    double timeSinceLastCall() {
        double currentTime = System.nanoTime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        return dt / 1e6;
    }
}
