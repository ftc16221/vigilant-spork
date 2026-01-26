package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.util.Global;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(group = Global.OpModeGroup.SETUP)
public class HomeSpindexer extends OpMode {

    public static double SEARCH_POWER = 0.1;
    public static float SENSOR_GAIN = 5f;

    public static float ALPHA_THRESHOLD = 0.96f;

    public static int RESET_DURATION = 100; // milliseconds

    Spindexer spindexer;

    MultipleTelemetry telemetryA;

    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;

    DcMotor spindexerMotor;

    State state = State.NOT_STARTED;
    Deadline resetDeadline = new Deadline(RESET_DURATION, TimeUnit.MILLISECONDS);

    @Override
    public void init() {
        spindexer = new Spindexer(this, new Intake(this));
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        colorSensor = spindexer.getColorSensor();
        colorSensor.setGain(SENSOR_GAIN);

        spindexerMotor = spindexer.getMotor();
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        switch (state) {
            case NOT_STARTED:
                spindexerMotor.setPower(SEARCH_POWER);
                state = State.SEARCHING;
                break;
            case SEARCHING:
                colors = colorSensor.getNormalizedColors();
                if (colors.alpha > ALPHA_THRESHOLD) {
                    spindexerMotor.setTargetPosition(spindexerMotor.getCurrentPosition());
                    spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    state = State.FOUND;
                }
                break;
            case FOUND:
                if (!spindexerMotor.isBusy()) {
                    spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetDeadline.reset();
                    state = State.SETTING;
                }
                break;
            case SETTING:
                if (resetDeadline.hasExpired()) {
                    spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    state = State.COMPLETE;
                }
                break;
            case COMPLETE:
                requestOpModeStop();
                break;
            case TUNING:
                colors = colorSensor.getNormalizedColors();
                telemetryA.addData("State", state);
                telemetryA.addData("Alpha", colors.alpha);
                telemetryA.addData("Alpha Threshold", ALPHA_THRESHOLD);
                telemetryA.update();
                break;
        }
    }

    enum State {
        NOT_STARTED, SEARCHING, FOUND, SETTING, COMPLETE, TUNING
    }
}
