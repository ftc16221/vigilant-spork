package org.firstinspires.ftc.teamcode.subassemblies

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.util.DashOpMode
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.teamcode.util.toDegrees
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sin

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
    @JvmField var CAMERA_POSITION = Position(DistanceUnit.INCH, TODO(), TODO(), TODO(), 0) // TODO: find where our camera is mounted on the robot
    @JvmField var CAMERA_ORIENTATION = YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -90.0, 0.0, 0)

    private val webcam = hardwareMap.get(WebcamName::class.java, "Webcam 1") // for our squirrel overlords
    val dash = DashOpMode.CameraStreamProcessor()

    val visionPortal = VisionPortal.Builder()
        .setCamera(webcam)
        .addProcessors(aprilTag, dash)
        .setCameraResolution(TODO())
        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
        .enableLiveView(false) // LiveView is only accessible if our control hub had a screen, or we plugged in an HDMI cable
        .setAutoStartStreamOnBuild(true)
        .build()

    // http://localhost:63342/RobotController/Vision-9.0.1-javadoc.jar/org/firstinspires/ftc/vision/apriltag/AprilTagProcessor.Builder.html
    val aprilTag = AprilTagProcessor.Builder()
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
        .build()

    val exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
    val ptzControl = visionPortal.getCameraControl(PtzControl::class.java)
    val panTiltHolder = PtzControl.PanTiltHolder()

    init {
        while(visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {}

        opMode.log("Vision successfully initialized")
    }

    /**
     * get all valid apriltag detections
     * @return ArrayList<AprilTagDetection>? list of valid apriltag detections (null if none)
     */
    fun getDetections(): ArrayList<AprilTagDetection>? {
        val currentDetections = aprilTag.detections
        val validDetections = ArrayList<AprilTagDetection>()
        for (detection in currentDetections) {
            if (detection.metadata != null) {
                validDetections.add(detection)
            } else {
                opMode.log("Detected invalid AprilTag, ID: ${detection.id}")
            }
        }
        validDetections.ifEmpty { return null }

        // Sort detections by distance from robot
        validDetections.sortBy { detection ->
            val position = detection.robotPose.position
            hypot(position.x.pow(2), position.y.pow(2))
        }

        return validDetections
    }

    /**
     * get the position of the robot on the field via AprilTags
     *
     * **be wary of this method, much of the math is AI generated because I no math right now**
     * @return field-centric position of the robot (null if no detections)
     */
    fun getPosition(): SparkFunOTOS.Pose2D? {
        val detection = getDetections()?.first()
        detection ?: return null

        val globalTagPosition = detection.metadata.fieldPosition
        val globalTagOrientation = detection.metadata.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS)
        val tagX = globalTagPosition[0].toDouble()
        val tagY = globalTagPosition[1].toDouble()
        val tagH = globalTagOrientation.thirdAngle.toDouble()

        val robotPose = detection.robotPose
        val robotX = robotPose.position.x
        val robotY = robotPose.position.y
        val robotH = robotPose.orientation.yaw

        // Calculate the field-centric position; AI generated, be wary of this
        val fieldX = tagX + robotX * cos(tagH) - robotY * sin(tagH)
        val fieldY = tagY + robotX * sin(tagH) + robotY * cos(tagH)
        val fieldH = tagH.toDegrees() + robotH

        return SparkFunOTOS.Pose2D(fieldX, fieldY, fieldH)
    }

    /**
     * find and return desired apriltag (based off ID)
     * @param id apriltag ID
     * @return AprilTagDetection? desired apriltag detection (null if not found)
     */
    fun findAprilTag(id: Int): AprilTagDetection? {
        val detections = getDetections()
        return detections?.find { it.id == id }
    }
}