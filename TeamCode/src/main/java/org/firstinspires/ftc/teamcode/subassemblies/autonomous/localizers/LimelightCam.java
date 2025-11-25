package org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers;

import static java.lang.Double.NaN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.CircularPoseArray;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.ArrayList;
import java.util.List;

@Config
public class LimelightCam extends Localizer {

    public static int POSE_ARRAY_SIZE = 15; // how many poses that should be stored and averaged
    public static int APRILTAG_PIPELINE_ID = 0;
    public static double LINEAR_TOLERANCE = 5; // cm
    public static double ANGULAR_TOLERANCE = 5;

    private final Limelight3A limelight3A;
    private final AprilTagLibrary tagLibrary;
    private final CircularPoseArray poseArray;

    private LLResult result;
    private int[] detectedTags;


    public LimelightCam(OpMode opMode) {
        super(opMode, "Limelight 3A");
        tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight3A.pipelineSwitch(APRILTAG_PIPELINE_ID);
        limelight3A.start();
        FtcDashboard.getInstance().startCameraStream(limelight3A, 60);
        poseArray = new CircularPoseArray(POSE_ARRAY_SIZE);
    }

    @Override
    public void update() {
        result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                // check if the new pose is close to the last ones, and if so use it, otherwise set pose to null
                Pose newPose = new Pose(result.getBotpose(), DistanceUnit.METER);
                poseArray.add(newPose);
                Pose avgPose = poseArray.getAverage();
                boolean withinTolerance =
                        avgPose.getDistanceFromPose(newPose) <= LINEAR_TOLERANCE
                        && avgPose.h - newPose.h <= ANGULAR_TOLERANCE;
                if (withinTolerance) {
                    pose = avgPose;
                } else {
                    pose = null;
                }
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
}