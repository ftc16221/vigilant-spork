package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Launcher;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp(group = Global.OpModeGroup.TEST)
public class PinpointDriftTest extends OpMode {

    public static int RPM_INCREMENT = 10;

    PinpointOdo pinpointOdo;
    Launcher launcher;

    MultipleTelemetry telemetryA;

    public static double flywheelVel = 0;

    @Override
    public void init() {
        pinpointOdo = new PinpointOdo(this, new Pose(0, 0, 0));
        launcher = new Launcher(this, new Spindexer(this, new Intake(this)));

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        pinpointOdo.update();
        launcher.update();

        if (gamepad1.dpad_up) {
            flywheelVel += RPM_INCREMENT;
        } else if (gamepad1.dpad_down) {
            flywheelVel -= RPM_INCREMENT;
        }
        launcher.setTargetVelocity(flywheelVel);

        telemetryA.addData("target RPM", flywheelVel);
        telemetryA.addData("actual RPM", launcher.getVelocity());
        telemetryA.addData("heading", pinpointOdo.getPose().h);
        telemetryA.update();
    }
}
