package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.PoseTracker;
import org.firstinspires.ftc.teamcode.util.DashOpMode;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp
@Config
public class BasicTeleOp extends LinearOpMode implements DashOpMode {

    public void runOpMode() {
        MecDriveBase driveBase = new MecDriveBase(this);
        Intake intake = new Intake(this);
        Underglow underglow = new Underglow(this);

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while(opModeIsActive()) {
                // DRIVEBASE
                driveBase.control(gamepad1);
                // INTAKE
                if (gamepad1.dpad_up || gamepad2.a) {
                    intake.run(Intake.Direction.IN);
                } else if (gamepad1.dpad_down || gamepad2.y) {
                    intake.run(Intake.Direction.OUT);
                } else if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad2.b) {
                    intake.stop();
                }
                // UPDATES
                telemetry.update();
            }
        }
    }
}
