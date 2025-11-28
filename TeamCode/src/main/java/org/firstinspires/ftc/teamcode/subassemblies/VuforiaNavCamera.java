// package org.firstinspires.ftc.teamcode.subassemblies;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// 
// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
// 
// public class VuforiaNavCamera {
// 
//     // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
//     private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
//     private static final boolean PHONE_IS_PORTRAIT = true;
// 
//     private static final String VUFORIA_KEY =
//         "Afbp4I3/////AAABmcEn57recUnKv/3EHsAO+jkAFD02oVRghZ8yX5DjgOIvkxO1ipG/fb3GeprYO+Bp6AVbmvW7ts21c71ldDDS8caXYkWTGpFaJ0CyMMfqJQnUabNsH7/sQjh99KlSOi+dOo75AuLUjuLO3nIksEFYpQ3Q8lAGl0ihH3npeTmO9X9KOTV2NJTNKEXZ3mXxBa8xEs9ZYhQy/ppkpExORmc6R+FJYnyykaTaFaXwdKg/R9LZnPQcLwuDD0EnoYlj74qOwVsekUfKxttKMb+FtFlgYm8pmXI5jqQdyidpSHUQn08G1EvqZBN/iuHWCDVhXP2zFRWcQdTEGltwsg47w/IJuLzFvsz04HEqyBz2Xh9eAbAn";
//     private LinearOpMode opMode;
// 
//     // Class Members
//     //public Navigation navigation = null;
//     public VuforiaCurrentGame vuforiaUltimateGoal;
//     public VuforiaBase.TrackingResults vuMarkResult;
// 
// //    public OpenGLMatrix lastLocation = null;
// //
// //    public VuforiaLocalizer vuforia = null;
// //    private VuforiaLocalizer.Parameters parameters = null;
//     private VuforiaTrackables targetsUltimateGoal = null;
// //    private boolean isStopRequested = false;
// 
//     private Telemetry telemetry = null;
// 
// //    public VuforiaNavCamera(LinearOpMode opMode) {
// //        // create Vuforia object
// //        vuforiaUltimateGoal = new VuforiaCurrentGame();
// //
// //
// //        vuforiaUltimateGoal.initialize(
// //                VUFORIA_KEY, // vuforiaLicenseKey
// //                CAMERA_CHOICE, // cameraDirection
// //                true, // useExtendedTracking
// //                true, // enableCameraMonitoring
// //                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
// //                0, // dx
// //                0, // dy
// //                0, // dz
// //                0, // xAngle
// //                0, // yAngle
// //                0, // zAngle
// //                true); // useCompetitionFieldTargetLocations
// //
// //        this.opMode = opMode;
// //    }
// 
// 
// 
//     /**
//      * Initialize the Vuforia localization engine. Copied from sample code.
//      */
// //    public VuforiaLocalizer initVuforia(int cameraMonitorViewId, WebcamName webcamName, Telemetry telemetry) {
// //
// //
// //
// //        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
// //        //parameters.cameraName = webcamName;
// //
// //        /*
// //        * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
// //        * This is how it was done in the TF Lite sample code, but we'd rather not instantiate lots of vuforias, I think.
// //        */
// //        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
// //
// //        parameters.vuforiaLicenseKey = VUFORIA_KEY;
// //        parameters.cameraDirection = CAMERA_CHOICE;
// //
// //        // Make sure extended tracking is disabled for this example.
// //        // Copied from Vuforia sample code. Not sure exactly what it does or how it will interact with TF lite.
// //        parameters.useExtendedTracking = false;
// //
// //        //  Instantiate the Vuforia engine
// //        vuforia = ClassFactory.getInstance().createVuforia(parameters);
// //
// //        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
// //
// //        return vuforia;
// //    }
// 
//     public void cleanupVuforia() {
// //        this.stopTrack();
//         // Disable Tracking when we are done;
//         vuforiaUltimateGoal.deactivate();
// //        targetsUltimateGoal.deactivate();
// //
// //        vuforia =  null;
// //        parameters = null;
// //        lastLocation = null;
//     }
// 
//     public VuforiaTrackables initTrackables(VuforiaLocalizer vuforia) {
//         // Copied from sample code.  We will only be playing red alliance side remote matches, most likely,
//         // but there's no reason to exclude the blue targets just in case we end up in a full arena
//         // or on the blue alliance side.
// 
//         // Load the data sets for the trackable objects. These particular data
//         // sets are stored in the 'assets' part of our application.
//         targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
//         VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
//         blueTowerGoalTarget.setName("Blue Tower Goal Target");
//         VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
//         redTowerGoalTarget.setName("Red Tower Goal Target");
//         VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
//         redAllianceTarget.setName("Red Alliance Target");
//         VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
//         blueAllianceTarget.setName("Blue Alliance Target");
//         VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
//         frontWallTarget.setName("Front Wall Target");
// 
//         /**
//          * In order for localization to work, we need to tell the system where each target is on the field, and
//          * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
//          * Transformation matrices are a central, important concept in the math here involved in localization.
//          * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
//          * for detailed information. Commonly, you'll encounter transformation matrices as instances
//          * of the {@link OpenGLMatrix} class.
//          *
//          * If you are standing in the Red Alliance Station looking towards the center of the field,
//          *     - The X axis runs from your left to the right. (positive from the center to the right)
//          *     - The Y axis runs from the Red Alliance Station towards the other side of the field
//          *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
//          *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
//          *
//          * Before being transformed, each target image is conceptually located at the origin of the field's
//          *  coordinate system (the center of the field), facing up.
//          */
// 
// //        //Set the position of the perimeter targets with relation to origin (center of field)
// //        redAllianceTarget.setLocation(OpenGLMatrix
// //                .translation(0, -halfField, mmTargetHeight)
// //                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
// //
// //        blueAllianceTarget.setLocation(OpenGLMatrix
// //                .translation(0, halfField, mmTargetHeight)
// //                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
// //        frontWallTarget.setLocation(OpenGLMatrix
// //                .translation(-halfField, 0, mmTargetHeight)
// //                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
// //
// //        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
// //        blueTowerGoalTarget.setLocation(OpenGLMatrix
// //                .translation(halfField, quadField, mmTargetHeight)
// //                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
// //        redTowerGoalTarget.setLocation(OpenGLMatrix
// //                .translation(halfField, -quadField, mmTargetHeight)
// //                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
// //
// //        setCameraLocationOnRobot(); //uses the trackables that have been saved to the instance.
// 
//         targetsUltimateGoal.activate();
//         return targetsUltimateGoal;
//     }
// 
// //    private void setCameraLocationOnRobot() {
// //        float phoneXRotate    = 0;
// //        float phoneYRotate    = 0;
// //        float phoneZRotate    = 0;
// //        //
// //        // Create a transformation matrix describing where the phone is on the robot.
// //        //
// //        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
// //        // Lock it into Portrait for these numbers to work.
// //        //
// //        // Info:  The coordinate frame for the robot looks the same as the field.
// //        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
// //        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
// //        //
// //        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
// //        // pointing to the LEFT side of the Robot.
// //        // The two examples below assume that the camera is facing forward out the front of the robot.
// //
// //        // We need to rotate the camera around it's long axis to bring the correct camera forward.
// //        if (CAMERA_CHOICE == VuforiaLocalizer.CameraDirection.BACK) {
// //            phoneYRotate = -90;
// //        } else {
// //            phoneYRotate = 90;
// //        }
// //
// //        // Rotate the phone vertical about the X axis if it's in portrait mode
// //        if (PHONE_IS_PORTRAIT) {
// //            phoneXRotate = 90 ;
// //        }
// //
// //        // Next, translate the camera lens to where it is on the robot.
// //        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
// //        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * Conversions.mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
// //        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * Conversions.mmPerInch;   // eg: Camera is 8 Inches above ground
// //        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
// //
// //        OpenGLMatrix robotFromCamera = OpenGLMatrix
// //                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
// //                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
// //
// //        /**  Let all the trackable listeners know where the phone is.  */
// //        for (VuforiaTrackable trackable : targetsUltimateGoal) {
// //            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
// //        }
// //
// //    }
// //
//     // uses instance data.
//     public OpenGLMatrix getRobotLocationFromVuMarks() {
//         // Note: To use the remote camera preview:
//         // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
//         // Tap the preview window to receive a fresh image.
//         OpenGLMatrix currentLocation = null;
//         boolean targetVisible = false;
// 
//         //targetsUltimateGoal.activate(); // moved to initTrackables
//         // run this until we actually get a location, or until told to stop.
//         while (currentLocation == null && opMode.opModeIsActive()) {
//             VuforiaBase.TrackingResults trackingResults = vuforiaUltimateGoal.track("RedTowerGoal");
//             if (trackingResults.isVisible) {
//                 //TODO: change this into a telemetry item that gets its value set.
//                 telemetry.addData("Visible Target", trackingResults.name);
//             }
//             // RedAlliance Target
//             // FrontWall Target
//             //TODO: Get robot location from tracked vuMark
//             //                    currentLocation = listener.getRobotLocation();
// 
// //            // check all the trackable targets to see which one (if any) is visible.
// //            for (VuforiaTrackable trackable : targetsUltimateGoal) {
// //                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
// //                if (listener.isVisible()) {
// //                    telemetry.addData("Visible Target", trackable.getName());
// //
// //                    // getUpdatedRobotLocation() will return null if no new information is available since
// //                    // the last time that call was made, or if the trackable is not currently visible.
// //                    // since we want this every time we call, then let's use getRobotLocation() instead.
// //                    currentLocation = listener.getRobotLocation();
// //                    break;
// //                } else {
// //                    telemetry.addData("No Visible Target", trackable.getName());
// //                }
// //            }
//         }
//         telemetry.addLine("Stop requested or we have a location.");
// 
//         return currentLocation;
//     }
// //
// //    public boolean stopTrack() {
// //        isStopRequested = true;
// //        return isStopRequested;
// //    }
// }
// 