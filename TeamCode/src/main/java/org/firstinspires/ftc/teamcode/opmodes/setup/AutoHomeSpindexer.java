package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.util.Global;

import java.util.concurrent.TimeUnit;


@TeleOp(group = Global.OpModeGroup.SETUP)
@Config
public class AutoHomeSpindexer extends OpMode {

    public static double HOMING_POWER = 0.2;
    public static long RESET_DURATION = 100;
    public static TimeUnit RESET_TIMEUNIT = TimeUnit.MILLISECONDS;

    private Spindexer spindexer;
    private DcMotor spindexerMotor;

    private State state = State.NOT_STARTED;
    private final Deadline resetTimer = new Deadline(RESET_DURATION, RESET_TIMEUNIT);

    @Override
    public void init() {
        spindexer = new Spindexer(this);
        spindexerMotor = spindexer.getMotor();
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        switch (state) {
            case NOT_STARTED: {
                if (isAtHome()) {
                    state = State.HAS_REACHED_HOME;
                } else {
                    spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    spindexerMotor.setPower(HOMING_POWER);
                    state = State.MOVING_TO_HOME;
                }
                break;
            }
            case MOVING_TO_HOME: {
                if (isAtHome()) {
                    int position = spindexerMotor.getCurrentPosition();
                    spindexerMotor.setTargetPosition(position);
                    spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    state = State.HAS_REACHED_HOME;
                }
                break;
            }
            case HAS_REACHED_HOME: {
                if (!spindexerMotor.isBusy()) {
                    spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetTimer.reset();
                    state = State.SETTING_HOME;
                }
                break;
            }
            case SETTING_HOME: {
                if (resetTimer.hasExpired()) {
                    spindexerMotor.setTargetPosition(0);
                    spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    state = State.HAS_SET_HOME;
                }
                break;
            }
            case HAS_SET_HOME: {
                requestOpModeStop();
                break;
            }
        }
        telemetry.addData("current state", state.toString());
    }

    private enum State {
        NOT_STARTED, MOVING_TO_HOME, HAS_REACHED_HOME, SETTING_HOME, HAS_SET_HOME
    }

    private boolean isAtHome() { return spindexer.getDetectedColor() == Spindexer.DetectedItem.HOME; }
}
