package org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers;

import static java.lang.Double.NaN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.ArrayList;
import java.util.List;

public class LimelightCam extends Localizer {

    private final Limelight3A limelight3A;
    private final AprilTagLibrary tagLibrary;

    private LLResult result;
    private int[] detectedTags;

    public LimelightCam(OpMode opMode) {
        super(opMode, "Limelight 3A");
        tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
        FtcDashboard.getInstance().startCameraStream(limelight3A, 60);
    }

    @Override
    public void update() {
        result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                pose = new Pose(result.getBotpose(), DistanceUnit.METER);
            } else {
                pose = null;
            }
        } else {
            pose = null;
        }

    }

    public List<Integer> getDetectedTagIds() {
        ArrayList<Integer> tagIds = new ArrayList<>();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult : fiducialResults) {
                tagIds.add(fiducialResult.getFiducialId());
            }
        }
        return tagIds;
    }

    public Pose getPoseOfTag(int aprilTagId) {
        AprilTagMetadata tagMetadata = tagLibrary.lookupTag(aprilTagId);
        return new Pose(tagMetadata.fieldPosition, tagMetadata.fieldOrientation);
    }

    public double getDistanceFromTag(int aprilTagId) {
        Pose tagPose = getPoseOfTag(aprilTagId);
        if (pose == null) return NaN;
        return pose.getDistanceFromPose(tagPose);
    }

    public LLResult getResult() {
        return result;
    }

    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        return result.getFiducialResults();
    }

    public enum Mode {
        APRILTAG, ARTIFACT_DETECTOR, PERSON_DETECTOR
    }
}
