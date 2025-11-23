package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Launcher;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.MathEx;

@TeleOp(group = Global.OpModeGroup.MAIN)
public class BasicTeleOp extends OpMode {

    public static double MAX_RPM = 6000;

    MecDriveBase driveBase;
    Intake intake;
    Launcher launcher;
    Underglow underglow;

    boolean dpadWasPressed = false;
    double targetRPM = 0;
    double prevTargetRPM = 0;

    @Override
    public void init() {
        driveBase = new MecDriveBase(this);
        intake = new Intake(this);
        launcher = new Launcher(this);
        underglow = new Underglow(this);

        telemetry.update();
    }

    @Override
    public void loop() {
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
        // LAUNCHER
        if (gamepad2.dpad_up && !dpadWasPressed) targetRPM += 100;
        else if (gamepad2.dpad_down && !dpadWasPressed) targetRPM -= 100;
        else if (gamepad2.dpad_right && !dpadWasPressed) targetRPM += 10;
        else if (gamepad2.dpad_left && !dpadWasPressed) targetRPM -= 10;

        dpadWasPressed = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_right || gamepad2.dpad_left;

        targetRPM = MathEx.clamp(targetRPM, -MAX_RPM, MAX_RPM);
        if (targetRPM != prevTargetRPM) {
            launcher.setTargetVelocity(targetRPM);
        }
        prevTargetRPM = targetRPM;

        launcher.update();

        telemetry.addData("Max RPM", MAX_RPM);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", launcher.getAverageVelocity());
        // UPDATES
        telemetry.update();
    }
}
