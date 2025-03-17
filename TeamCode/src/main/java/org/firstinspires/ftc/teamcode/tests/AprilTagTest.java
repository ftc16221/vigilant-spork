package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.subassemblies.Vision;

@Disabled
@TeleOp(name = "AprilTag Test", group = "tests")
public class AprilTagTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Vision vision = new Vision(this);
        ExposureControl exposureControl = vision.getVisionPortal().getCameraControl(ExposureControl.class);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addLine("AprilTag Test");
                telemetry.addData("AprilTags Detected", vision.getDetections() == null ? 0 : vision.getDetections().size());
                telemetry.update();
            }
        }
    }
}