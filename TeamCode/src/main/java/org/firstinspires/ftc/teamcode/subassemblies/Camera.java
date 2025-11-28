 package org.firstinspires.ftc.teamcode.subassemblies;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.HardwareMap;
 import com.qualcomm.robotcore.util.RobotLog;

 import org.firstinspires.ftc.robotcore.external.Telemetry;
 import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
 import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
 import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
 import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
 import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
 import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
 //import org.firstinspires.ftc.teamcode.navigation.Destination;
 //import org.firstinspires.ftc.teamcode.navigation.FieldDestinations;
 import org.firstinspires.ftc.teamcode.util.RecognitionMatrix;

 import java.util.List;

 public class Camera {

     public WebcamName webCam1;

     // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
     private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
     private static final boolean PHONE_IS_PORTRAIT = true;

     private static final String VUFORIA_KEY =
             "Afbp4I3/////AAABmcEn57recUnKv/3EHsAO+jkAFD02oVRghZ8yX5DjgOIvkxO1ipG/fb3GeprYO+Bp6AVbmvW7ts21c71ldDDS8caXYkWTGpFaJ0CyMMfqJQnUabNsH7/sQjh99KlSOi+dOo75AuLUjuLO3nIksEFYpQ3Q8lAGl0ihH3npeTmO9X9KOTV2NJTNKEXZ3mXxBa8xEs9ZYhQy/ppkpExORmc6R+FJYnyykaTaFaXwdKg/R9LZnPQcLwuDD0EnoYlj74qOwVsekUfKxttKMb+FtFlgYm8pmXI5jqQdyidpSHUQn08G1EvqZBN/iuHWCDVhXP2zFRWcQdTEGltwsg47w/IJuLzFvsz04HEqyBz2Xh9eAbAn";


     private TensorFlowObjectDetector tfod;

     //public VuforiaNavCamera vuforia;
     private LinearOpMode opMode;
     private VuforiaCurrentGame vuforiaUltimateGoal;

     public Camera(HardwareMap hardwareMap, LinearOpMode opMode) {
         /**
          * Copied from Copied from sample code - this went at the beginning of the OpMode to initialize the camera views.
          **/

         this.opMode = opMode;
         this.telemetry = opMode.telemetry;

         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

         int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                 "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

         initVision(cameraMonitorViewId, tfodMonitorViewId, opMode);
     }
     // keep this instance safe; we don't want to update it once we have it.
     private RecognitionMatrix detectedRingStack;
     public RecognitionMatrix getRingStack() {
         if (detectedRingStack == null) {
             detectedRingStack = detectRingsUsingTensorFlow();
         }
         return detectedRingStack;
     }

     //public VuforiaNavCamera vuforiaForTfod = new VuforiaNavCamera();
     //public TensorFlowObjectDetector tensorflow = new TensorFlowObjectDetector();
     public VuforiaTrackables vuMarkTargets = null;

     public Telemetry telemetry = null;

     //public int cameraMonitorViewId = 0;
     //public int tfodMonitorViewId = 0;

     public int getCountOfRings(RecognitionMatrix ringStack) {
         if(ringStack == null) { // No rings detected
             return 0;
         }
         switch(ringStack.recognition.getLabel()) {
             case "Single":
                 return 1;
             case "Quad":
                 return 4;
             default:
                 return 0;
         }
     }
     public int getCountOfRings() {
         return getCountOfRings(getRingStack());
     }

 //    public Destination getTargetZone(int rings) {
 //        switch(rings) {
 //            case 1:
 //                return FieldDestinations.TZB;
 //            case 4:
 //                return FieldDestinations.TZC;
 //            default:
 //                return FieldDestinations.TZA;
 //        }
 //    }
 //    public Destination getTargetZone(RecognitionMatrix recog) {
 //        return getTargetZone(getCountOfRings(recog));
 //    }
 //    public Destination getTargetZone() {
 //        return getTargetZone(getCountOfRings());
 //    }

     public RecognitionMatrix detectRingsUsingTensorFlow() {
         // implement detectRings using TensorFlowLite (built in to TFL)
         List<Recognition> recognitions = tfod.getRecognitions();
         if (recognitions == null) { return null; }

         tfod.printRecognitions(recognitions, telemetry);
         List<Recognition> recognizedRingStacks = tfod.recognitionsByLabel(recognitions, "Quad");
         recognizedRingStacks.addAll(tfod.recognitionsByLabel(recognitions, "Single"));
         if(recognizedRingStacks.size() > 0) {
             Recognition first = recognizedRingStacks.get(0);
             RobotLog.i("*16221 Camera*", "Recognized some ring(s): [" + first.getLabel() + "] (" + first.getConfidence() + ")");
             return new RecognitionMatrix(first);
         } else {
             return null;
         }
     }

     public OpenGLMatrix detectWobbleGoalUsingTensorFlow() {
         // TODO: implement wobble goal detection using TensorFlowLite (custom model)
         return null;
     }

 //    public OpenGLMatrix getRobotLocationUsingVuMarks() {
 //        return vuforia.getRobotLocationFromVuMarks();
 //    }

     public VectorF getRobotTranslationUsingVuMarks() {
         return null; //Navigation.getTranslationVector(vuforia.getRobotLocationFromVuMarks(), telemetry);
     }

     public void initVision(int cameraMonitorViewId, int tfodMonitorViewId, LinearOpMode opMode) {
         // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
         // first, then the object detector.
         /*
         VuforiaLocalizer vl2 = vuforiaForTfod.initVuforia(cameraMonitorViewId, webCam2);
         TFObjectDetector tfod = tensorflow.initTfod(tfodMonitorViewId, vl2);
          */
 //        vuforia = new VuforiaNavCamera(opMode);

         // create Vuforia object
         vuforiaUltimateGoal = new VuforiaCurrentGame();


         vuforiaUltimateGoal.initialize(
                 VUFORIA_KEY, // vuforiaLicenseKey
                 CAMERA_CHOICE, // cameraDirection
                 true, // useExtendedTracking
                 true, // enableCameraMonitoring
                 VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                 0, // dx
                 0, // dy
                 0, // dz
                 0, // xAngle
                 0, // yAngle
                 0, // zAngle
                 true); // useCompetitionFieldTargetLocations


         // The other camera is for navigation using vumarks.  We will load all trackables to it.
         VuforiaLocalizer vl1 = vuforiaUltimateGoal.getVuforiaLocalizer(); // vuforia.initVuforia(cameraMonitorViewId, webCam1, telemetry);
         //vuMarkTargets = vuforia.initTrackables(vl1);

         tfod = new TensorFlowObjectDetector(tfodMonitorViewId, vl1);
         // tfod.initTfod(tfodMonitorViewId, vl1);
     }

     public void cleanupVision() {
         vuforiaUltimateGoal.deactivate();
         //vuforia.cleanupVuforia();
         tfod.cleanupTfod();
         vuforiaUltimateGoal = null;
         vuMarkTargets = null;
         this.telemetry.addLine("Cleaning up vision functionality.");
     }


 }
