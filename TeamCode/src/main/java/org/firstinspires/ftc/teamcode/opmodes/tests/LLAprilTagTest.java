package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.util.Global;

@Disabled
@TeleOp(group = Global.OpModeGroup.TEST)
public class LLAprilTagTest extends OpMode {

    LimelightCam limelight;
    MultipleTelemetry telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void init() {
        limelight = new LimelightCam(this);
    }

    @Override
    public void loop() {
        limelight.update();

        LLResult result = limelight.getResult();
        telemetryA.addData("isValid", result.isValid());
        if (result.isValid()) {
            telemetryA.addData("tx", result.getTx());
            telemetryA.addData("ty", result.getTy());
            telemetryA.addData("ta", result.getTa());
            telemetryA.addData("botPose", result.getBotpose());
            telemetryA.addData("botPoseAvgArea", result.getBotposeAvgArea());
            telemetryA.addData("botPoseAvgDist", result.getBotposeAvgDist());
        }
        telemetryA.addData("detectedTagIds", limelight.getDetectedTagIds().toString());

        if (!result.getFiducialResults().isEmpty()) {
            telemetryA.addData("robot pose relative to field coordinate system", result.getFiducialResults().get(0).getRobotPoseFieldSpace());
        }
    }
}
