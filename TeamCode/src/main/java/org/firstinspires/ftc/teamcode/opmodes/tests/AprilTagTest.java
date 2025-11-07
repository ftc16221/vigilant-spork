package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.GenericCam;

@Disabled
@TeleOp(group = "tests")
public class AprilTagTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GenericCam vision = new GenericCam(this);
        ExposureControl exposureControl = vision.getVisionPortal().getCameraControl(ExposureControl.class);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addLine("AprilTag Test");
                telemetry.addData("AprilTags Detected", vision.getValidDetections() == null ? 0 : vision.getValidDetections().size());
                telemetry.update();
            }
        }
    }
}