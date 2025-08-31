package org.firstinspires.ftc.teamcode.subassemblies

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.util.DashOpMode
import org.firstinspires.ftc.teamcode.util.Global
import org.firstinspires.ftc.teamcode.util.Pose
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

/**
 * Vision subassembly for detecting AprilTags and calculating robot position on the field via AprilTags.
 * Possibly will be used for Machine Learning in future seasons
 */
@Config
class Vision(opMode: OpMode): Subassembly(opMode, "Vision") {
    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */

    @Config
    companion object {
        @JvmField
        var CAMERA_POSITION = Position(DistanceUnit.INCH, 1.45, 6.75, 4.75, 0)
        @JvmField
        var CAMERA_ORIENTATION = YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -90.0, 0.0, 0)
        @JvmField
        var CAMERA_RESOLUTION = Size(1280, 720)

        @JvmField
        var APRILTAG_RANGE_LIMIT = 60.0; // inches

        @JvmField
        var BLUE_APRILTAG_IDS = listOf(11, 12, 13)
        @JvmField
        var RED_APRILTAG_IDS = listOf(14, 15, 16)

        // camera calibration values
        @JvmField
        var FX = 484.01521684
        @JvmField
        var FY = 484.01521684
        @JvmField
        var CX = 682.931618492
        @JvmField
        var CY = 339.304216996
    }

    private val webcam =
        hardwareMap.get(WebcamName::class.java, "Webcam 1") // for our squirrel overlords
    val dash = DashOpMode.CameraStreamProcessor()

    // http://localhost:63342/RobotController/Vision-9.0.1-javadoc.jar/org/firstinspires/ftc/vision/apriltag/AprilTagProcessor.Builder.html
    val aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
        .setLensIntrinsics(FX, FY, CX, CY)
        .build()

    val visionPortal = VisionPortal.Builder()
        .setCamera(webcam)
        .addProcessors(aprilTag, dash)
        .setCameraResolution(CAMERA_RESOLUTION)
        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
        .enableLiveView(false) // LiveView is only accessible if our control hub had a screen, or we plugged in an HDMI cable
        .setAutoStartStreamOnBuild(true)
        .build()

// this stuff was breaking everything so i commented it out
//    val exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
//    val ptzControl = visionPortal.getCameraControl(PtzControl::class.java)
//    val panTiltHolder = PtzControl.PanTiltHolder()

    init {
        while (visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {
            // intentionally blank
        }

        val exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
//        exposureControl.setExposure()
        val ptzControl = visionPortal.getCameraControl(PtzControl::class.java)
        val panTiltHolder = PtzControl.PanTiltHolder()

        FtcDashboard.getInstance().startCameraStream(dash, 0.0)

        opMode.log("Vision successfully initialized")
    }

    /**
     * get all valid apriltag detections
     * @return ArrayList<AprilTagDetection>? list of valid apriltag detections (null if none)
     */
    fun getValidDetections(): ArrayList<AprilTagDetection>? {
        val currentDetections = aprilTag.detections
        val validDetections = ArrayList<AprilTagDetection>()
        for (detection in currentDetections) {
            if (detection.metadata != null) {
                validDetections.add(detection)
            } else {
                opMode.log("Detected invalid AprilTag, ID: ${detection.id}")
            }
        }

        validDetections.retainAll { it.ftcPose.range < APRILTAG_RANGE_LIMIT }

        validDetections.ifEmpty { return null }

        // Sort detections by distance from robot
        validDetections.sortBy { it.ftcPose.range }

        return validDetections
    }

    /**
     * get the position of the robot on the field via AprilTags
     * @return field-centric position of the robot (null if no detections)
     */
    fun getPosition(): SparkFunOTOS.Pose2D? {
        val detection = getValidDetections()?.first()
        detection ?: return null

        val position = detection.robotPose.position
        val orientation = detection.robotPose.orientation

        // values are all weird because FTC hates standardization
        val x = -position.y
        val y = position.x
        val h = orientation.yaw + 90

        val rawPose = SparkFunOTOS.Pose2D(x, y, h)

        return when (Global.alliance) {
            Global.Alliance.BLUE -> rawPose
            Global.Alliance.RED -> rawPose.invert()
            null -> {
                if (RED_APRILTAG_IDS.contains(detection.id)) rawPose.invert()
                else rawPose
            }
        }
    }

    fun getPositionIsNotNull() = getPosition() != null

    fun getPose(): Pose { return Pose(getPosition()) }
    /**
     * find and return desired apriltag (based off ID)
     * @param id apriltag ID
     * @return AprilTagDetection? desired apriltag detection (null if not found)
     */
    fun findAprilTag(id: Int): AprilTagDetection? {
        val detections = getValidDetections()
        return detections?.find { it.id == id }
    }

    fun SparkFunOTOS.Pose2D.invert() = SparkFunOTOS.Pose2D(-x, -y, h + 180)
}