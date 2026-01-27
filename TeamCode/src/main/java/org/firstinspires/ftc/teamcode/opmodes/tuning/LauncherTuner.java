package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Intake;
import org.firstinspires.ftc.teamcode.subassemblies.Launcher;
import org.firstinspires.ftc.teamcode.subassemblies.Spindexer;
import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.MathEx;
import org.firstinspires.ftc.teamcode.util.ToggleServo;

@TeleOp(group = Global.OpModeGroup.TUNER)
@Config
public class LauncherTuner extends OpMode {

    public static double TARGET_RPM = 0;
    public static double MAX_RPM = 6000;
    public static boolean USE_LIMELIGHT = false;
    public static int GOAL_APRILTAG_ID = 24; // default: tag id of red goal. blue goal is id 20

    private double prevTargetRPM = 0;
    private final boolean useLimelight = USE_LIMELIGHT;

    private boolean dpadWasPressed = false;
    double prevTime = 0;

    Launcher launcher;
    LimelightCam limelightCam;
    Watchdog watchdog;

    ToggleServo gateServo;

    MultipleTelemetry telemetryA;

    @Override
    public void init() {

        launcher = new Launcher(this, new Spindexer(this, new Intake(this)));
        gateServo = launcher.getGateServo();

        if (useLimelight) {
            limelightCam = new LimelightCam(this);
        }

        watchdog = new Watchdog(this);

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.addLine(
                "This OpMode is built to help find the A, B, and C values for the flywheel Launcher's quadratic. A table should be kept, recording target rpm, actual rpm, and distance from goal." +
                        "Then, those values should be plotted in Desmos, and a quadratic function of best fit should be found. The A, B, and C values should be recorded and permanently added to Launcher.java\n" +
                        "to change target rpm either do it directly through FTC dashboard (as I recommend) or use the dpad to increment rpm using up/down to increment by 100, and left/right to increment by 10."
        );
    }

    @Override
    public void start() {
        telemetryA.clear();
    }

    @Override
    public void loop() {
        double dt = time - prevTime;
        prevTime = time;

        if (gamepad1.dpad_up && !dpadWasPressed) TARGET_RPM += 100;
        else if (gamepad1.dpad_down && !dpadWasPressed) TARGET_RPM -= 100;
        else if (gamepad1.dpad_right && !dpadWasPressed) TARGET_RPM += 10;
        else if (gamepad1.dpad_left && !dpadWasPressed) TARGET_RPM -= 10;

        dpadWasPressed = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left;

        if (gamepad1.aWasPressed()) {
            launcher.setHoodAngle(0);
        } else if (gamepad1.yWasPressed()) {
            launcher.setHoodAngle(45);
        } else if (gamepad1.xWasPressed()) {
            launcher.setHoodAngle(60);
        }

        if (gamepad1.right_trigger > 0.1) {
            gateServo.open();
        } else if (gamepad1.left_trigger > 0.1) {
            gateServo.close();
        }

        TARGET_RPM = MathEx.clamp(TARGET_RPM, -MAX_RPM, MAX_RPM);
        if (TARGET_RPM != prevTargetRPM) {
            launcher.setTargetVelocity(TARGET_RPM);
        }
        prevTargetRPM = TARGET_RPM;

        telemetryA.addData("Max RPM", MAX_RPM);
        telemetryA.addData("Target RPM", TARGET_RPM);
        telemetryA.addData("Actual RPM", launcher.getVelocity());
        if (useLimelight) {
            limelightCam.update();
            telemetryA.addData("Distance from goal", limelightCam.getDistanceFromTag(GOAL_APRILTAG_ID));
        }
        telemetryA.addData("loop time", dt);

        launcher.update();
    }
}

