package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.util.Global;

import java.util.concurrent.TimeUnit;


@TeleOp(group = Global.OpModeGroup.SETUP)
@Config
public class SetSpindexerHome extends OpMode {

    public static long RESET_DURATION = 100; // milliseconds

    private Spindexer spindexer;
    private DcMotor spindexerMotor;

    private final Deadline resetTimer = new Deadline(RESET_DURATION, TimeUnit.MILLISECONDS);

    @Override
    public void init() {
        spindexer = new Spindexer(this, new Intake(this));
        spindexerMotor = spindexer.getMotor();
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        resetTimer.reset();
    }

    @Override
    public void loop() {
        if (resetTimer.hasExpired()) {
            spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            requestOpModeStop();
        }
    }
}
