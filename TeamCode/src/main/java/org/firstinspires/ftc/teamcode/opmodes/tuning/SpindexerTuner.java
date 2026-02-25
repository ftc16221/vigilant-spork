package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.TUNER)
public class SpindexerTuner extends OpMode {

    public static double spindexerAngle = 0;

    Spindexer spindexer;
    DcMotor spindexerMotor;

    @Override
    public void init() {
        spindexer = new Spindexer(this, new Intake(this));
        spindexerMotor = spindexer.getMotor();
    }

    @Override
    public void start() {
        spindexerMotor.setPower(0.8);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed() || gamepad1.dpad_right) spindexerAngle += 1;
        if (gamepad1.dpadDownWasPressed() || gamepad1.dpad_left) spindexerAngle -= 1;

        double error = spindexerAngle - spindexer.getCurrentAngle();
        spindexer.evaluatePID(error);

        telemetry.addData("target spindexer position", spindexerAngle);
        telemetry.addData("current spindexer position", spindexer.getCurrentAngle());
    }
}
