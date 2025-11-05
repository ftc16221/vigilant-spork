package org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

public class LimelightCam extends Localizer {

    private final Limelight3A limelight3A;
    private final AprilTagLibrary tagLibrary;

    public LimelightCam(OpMode opMode) {
        super(opMode, "Limelight 3A");
        tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
    }

    @Override
    public void update() {
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                pose = new Pose(result.getBotpose());
            }
        } else {
            pose = null;
        }
    }

    public Pose getPoseOfTag(int aprilTagId) {
        AprilTagMetadata tagMetadata = tagLibrary.lookupTag(aprilTagId);
        Orientation tagOrientation = Orientation.getOrientation(tagMetadata.fieldOrientation.toMatrix(), AxesReference.INTRINSIC, AxesOrder.XYZ, Global.ANGLE_UNIT);
        double heading = tagOrientation.thirdAngle; // third angle is Z, yaw/heading
        return new Pose(tagMetadata.fieldPosition, heading);
    }

    public double getDistanceFromTag(int aprilTagId) {
        Pose tagPose = getPoseOfTag(aprilTagId);
        return pose.getDistanceFromPose(tagPose);
    }
}
