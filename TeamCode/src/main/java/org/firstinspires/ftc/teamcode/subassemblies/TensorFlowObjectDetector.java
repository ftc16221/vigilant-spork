 package org.firstinspires.ftc.teamcode.subassemblies;

 import com.qualcomm.robotcore.util.RobotLog;

 import org.firstinspires.ftc.robotcore.external.ClassFactory;
 import org.firstinspires.ftc.robotcore.external.Telemetry;
 import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
 import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
 import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

 import java.util.ArrayList;
 import java.util.List;

 public class TensorFlowObjectDetector {

     // TensorFlow Lite Object Detection
     private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
     private static final String LABEL_FIRST_ELEMENT = "Quad";
     private static final String LABEL_SECOND_ELEMENT = "Single";

     public TFObjectDetector tfod = null;

     public TensorFlowObjectDetector(int tfodMonitorViewId, VuforiaLocalizer vuforiaLocalizer) {
         this.tfod = initTfod(tfodMonitorViewId, vuforiaLocalizer);
     }
     /**
      * Initialize the TensorFlow Object Detection engine. Copied from sample code, altered to use injected instance.
      */
     public TFObjectDetector initTfod(int tfodMonitorViewId, VuforiaLocalizer vuforia) {

         TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
         tfodParameters.minResultConfidence = 0.8f;
 //        tfodParameters.
         tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
         // this loads the model for the rings.
         tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
         // to detect the wobble goal, we will need a custom model.


         /**
          * Activate TensorFlow Object Detection before we wait for the start command.
          * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
          **/
         if (tfod != null) {
             tfod.activate();

             // The TensorFlow software will scale the input images from the camera to a lower resolution.
             // This can result in lower detection accuracy at longer distances (> 55cm or 22").
             // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
             // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
             // should be set to the value of the images used to create the TensorFlow Object Detection model
             // (typically 16/9).
             tfod.setZoom(2.5, 16.0/9.0);
         }

         return tfod;
     }

     public List<Recognition> getRecognitions() {
         if(tfod != null) {
             RobotLog.i("Getting recognitions.");
             return tfod.getRecognitions();
             // return tfod.getUpdatedRecognitions();
             // this will return null if no new recognitions are found.
             // Handle null in caller!
         } else {
             RobotLog.i("TFOD is null, returning no recognitions.");
             return null;
         }
     }

     public void printRecognitions(List<Recognition> recognitions, Telemetry telemetry) {
         if(recognitions != null) {
             telemetry.addData("# Objects", recognitions.size());
             int i = 0;
             for(Recognition recognition : recognitions) {
                 telemetry.addData(String.format("Label (%d)", i), recognition.getLabel());
                 telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                         recognition.getLeft(), recognition.getTop());
                 telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                         recognition.getRight(), recognition.getBottom());
                 RobotLog.i("Recognition: " + recognition.getLabel() + " with confidence " + recognition.getConfidence() +
                         " at (left " + recognition.getLeft() + ", top " + recognition.getTop() +
                         ", bottom " + recognition.getBottom() + ",right " + recognition.getRight() + ")");
             }
             telemetry.update();
         } else {
             RobotLog.i("Recognitions list is null.");
         }
     }
     public List<Recognition> recognitionsByLabel(List<Recognition> recognitions, String label) {
         List<Recognition> result = new ArrayList<Recognition>();
         if(recognitions != null) {
             for(Recognition recognition : recognitions) {
                 if(recognition.getLabel() == label) {
                     RobotLog.i("recognition with label " + label + " found.");
                     result.add(recognition);
                 }
             }
         }
         return result;

     }
     public void cleanupTfod() {
         if(tfod != null) {
             tfod.shutdown();
             this.tfod = null;
         }
     }
 }
