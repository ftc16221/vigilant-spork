package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Launcher;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.MathKt;

@TeleOp(group = Global.OpModeGroup.TUNER)
@Config
public class LauncherTuner extends OpMode {

    public static double TARGET_RPM = 0;
    public static double MAX_RPM = 6000;
    public static boolean USE_LIMELIGHT = false;
    public static int GOAL_APRILTAG_ID = 24; // default: tag id of red goal. blue goal is id 20

    private double prevTargetRPM = 0;
    private final boolean useLimelight = USE_LIMELIGHT;

    Launcher launcher;
    LimelightCam limelightCam;

    MultipleTelemetry telemetryA;

    @Override
    public void init() {

        launcher = new Launcher(this);

        if (useLimelight) {
            limelightCam = new LimelightCam(this);
        }

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
        if (gamepad1.dpadUpWasPressed()) TARGET_RPM += 100;
        if (gamepad1.dpadDownWasPressed()) TARGET_RPM -= 100;
        if (gamepad1.dpadRightWasPressed()) TARGET_RPM += 10;
        if (gamepad1.dpadLeftWasPressed()) TARGET_RPM -= 10;

        MathKt.clamp(TARGET_RPM, -MAX_RPM, MAX_RPM);
        if (TARGET_RPM != prevTargetRPM) {
            launcher.setTargetRPM(TARGET_RPM);
        }
        prevTargetRPM = TARGET_RPM;


        telemetryA.addData("Max RPM", MAX_RPM);
        telemetryA.addData("Target RPM", TARGET_RPM);
        telemetryA.addData("Actual RPM", launcher.getCurrentRPM());
        if (useLimelight) {
            telemetryA.addData("Distance from goal", limelightCam.getDistanceFromTag(GOAL_APRILTAG_ID));
        }
    }
}

