 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.Blinker;
 import com.qualcomm.robotcore.hardware.ColorSensor;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DistanceSensor;
 import com.qualcomm.robotcore.hardware.OrientationSensor;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.Axis;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
 import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
 import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
 import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
 import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

 import java.util.ArrayList;
 import java.util.List;

 @Autonomous(name = "PIDAuto1")

     // todo: write your code here


 public class PIDAuto1 extends LinearOpMode {

   private VuforiaCurrentGame vuforiaUltimateGoal;
   private Blinker expansion_Hub_1;
   private Blinker expansion_Hub_2;
   private DistanceSensor front;
   private DistanceSensor rear;
   private DistanceSensor left;
   private DistanceSensor right;
   private ColorSensor color_sensor;
   private DcMotor FLDrive;
   private DcMotor FRDrive;
   private DcMotor BLDrive;
   private DcMotor BRDrive;
   private BNO055IMU imu;

   // Define and Initialize Motors

         //leftArm.setPower(0);

   /**
    * This function is executed when this Op Mode is selected from the Driver Station.
    */


   //public void init() {
         /* Initialize the hardware variables.
          * The init() method of the hardware class does all the work here
          */
         //robot.init(hardwareMap);

         // Send telemetry message to signify robot waiting;
   //      telemetry.addData("Say", "Hello Driver");
         //
   //}


   public class Destination {

       //declare variables for name and and coordinates array
       public String destName;
       private float[] destCoord;

       //constructor
       public Destination(String name,float destX,float destY){
         destCoord = new float[2];
         destName = name;
         destCoord[0] = destX;
         destCoord[1] = destY;
       }
   }

   public void runOpMode() {
     VuforiaBase.TrackingResults vuMarkResult;
     float X;
     float Y;
     float Z;
     float Xangle;
     float Yangle;
     float Zangle;
     int autoStage = 0;

     BNO055IMU.Parameters imuParameters;
     Quaternion quat;
     float q0;
     float q1;
     float q2;
     float q3;
     double psi;
     double theta;
     double phi;
     Orientation angles;
     List<Axis> foo;
     Acceleration gravity;

     double errX = 0;
     double errY = 0;
     double errZ = 0;
     double errX_prev = 0;
     double errY_prev = 0;
     double errZ_prev = 0;
     double dvErrX = 0;
     double dvErrY = 0;
     double dvErrZ = 0;
     double iErrX = 0;
     double iErrY = 0;
     double iErrZ = 0;

     double Kp = 0.006;
     double Kd = -0.03;
     double Ki = 0.000005;

     imu = hardwareMap.get(BNO055IMU.class, "imu");
     double FLMotorPwr = 0;
     double FRMotorPwr = 0;
     double BLMotorPwr = 0;
     double BRMotorPwr = 0;
     double msTime_now = 0;
     double msTime_prev = 0;
     double dTime = 0;

     double maxMtrPwr = 0.25;

     Destination nextDest = new Destination("launchLine",620,0);

     ElapsedTime msTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

     FLDrive  = hardwareMap.dcMotor.get("left_front");
     FRDrive  = hardwareMap.dcMotor.get("right_front");
     BLDrive  = hardwareMap.dcMotor.get("left_rear");
     BRDrive  = hardwareMap.dcMotor.get("right_rear");

     // Set all motors to zero power
     FLDrive.setPower(0);
     FRDrive.setPower(0);
     BLDrive.setPower(0);
     BRDrive.setPower(0);

     FLDrive.setDirection(DcMotor.Direction.REVERSE);
     FRDrive.setDirection(DcMotor.Direction.FORWARD);
     BLDrive.setDirection(DcMotor.Direction.REVERSE);
     BRDrive.setDirection(DcMotor.Direction.FORWARD);


     vuforiaUltimateGoal = new VuforiaCurrentGame();

     // Initialize Vuforia (use default settings).
     vuforiaUltimateGoal.initialize(
         "Ae1X2rH/////AAABmeorpS5ODUuhnNQj0tniyedqUdW5kd2tWpw0i4PSCg4cPvqgm+m+GK+y2xUjIy1fOTLwE3zvW45TabFJ/IbwJuyW7X5dqX04Y4lVpHC82Xs+ZMycEUe43yR7qCD387etZbPK3trbZgJ3ZOG+LbSgpJVxsBVb9s5cDXa9MGzHaxNFbqNxqCDGVMeqADDMzgaYNQRf4QlM/91ss6P45q84o88Eo3S89LevDSVwunBLKlSUoLEwTJh4oNfAzVN5oxggCzA5fdFsgyzZ4Xo8Ud/VEh+RmdXNalAcRfzyqRX/9oE1cLSEEvvQEaZmHjXxGIZ9WrDb84ibNiVmLNHHuCh4/+ZXU6fZ0ceFwfeJwY1UZivO", // vuforiaLicenseKey
         VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
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

     // Create new IMU Parameters object.
     imuParameters = new BNO055IMU.Parameters();
     //  set mode to "IMU". This will use the
     // device's gyro and its accelerometer
     // to calculate the relative orientation of the
     // hub and therefore the robot.
     // Gyro gets calibrated automatically in this mode.
     imuParameters.mode = BNO055IMU.SensorMode.NDOF;
     // Use degrees as angle unit.
     imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
     // Express acceleration as m/s^2.
     imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
     // Set acceleration integration algorithm to
     // "naive" which will give velocity & position
     imuParameters.accelerationIntegrationAlgorithm = null;
     // Enable logging.
     imuParameters.loggingEnabled = false;
     // Initialize IMU.
     imu.initialize(imuParameters);
     // Initialize variables.
     // Report the initialization to the Driver Station
     telemetry.addData("IMU status", "IMU Initalized.  Calibration started....");

     // Prompt user to push start button.
     telemetry.addData("PID with VuMark", "Press start to continue...");
     telemetry.update();
     // Wait until user pushes start button.
     waitForStart();

     msTime.reset();

     if (opModeIsActive()) {
       // Activate Vuforia software.
       vuforiaUltimateGoal.activate();
       while (opModeIsActive()) {
         // Get the tracking results.
         vuMarkResult = vuforiaUltimateGoal.track("Red Tower Goal Target");
         // Is a VuMark visible?
         if (vuMarkResult.isVisible) {
           // Yes, we see one.
           msTime_prev = msTime_now;
           msTime_now = msTime.time();
           dTime = msTime_now - msTime_prev;

           telemetry.addData("VuMark", "A VuMark is visible.");

           X = vuMarkResult.x;
           Y = vuMarkResult.y;
           Z = vuMarkResult.z;
           Xangle = vuMarkResult.xAngle;
           Yangle = vuMarkResult.yAngle;
           Zangle = vuMarkResult.zAngle;

           quat = imu.getQuaternionOrientation();
           q0 = quat.w;
           q1 = quat.x;
           q2 = quat.y;
           q3 = quat.z;
           // convert quaternion to Euler angles.
           // Since the IMU's parameters were
           // set up for DEGREES, then the
           // atan2 functions here will return
           // values in DEGREES, not radians
           // psi = yaw (or heading)
           psi = Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) / Math.PI * 180;
           // theta = pitch
           theta = Math.asin(2 * (q0 * q2 - q3 * q1)) / Math.PI * 180;
           // phi = roll
           phi = Math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) / Math.PI * 180;
           // post results
           angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
           foo = new ArrayList<Axis>(((OrientationSensor) imu).getAngularOrientationAxes());
           gravity = imu.getGravity();


           errX_prev = errX;
           errY_prev = errY;
           errX = X - nextDest.destCoord[0];
           errY = Y - nextDest.destCoord[1];
           dvErrX = (errX - errX_prev)/dTime;
           dvErrY = (errY - errY_prev)/dTime;
           iErrX = iErrX + errX*dTime;
           iErrY = iErrY + errY*dTime;

           FLMotorPwr = -maxMtrPwr*((Kp * errX) + (Kd * dvErrX) + (Ki * iErrX));
           FRMotorPwr = -maxMtrPwr*((Kp * errX) + (Kd * dvErrX) + (Ki * iErrX));
           BLMotorPwr = -maxMtrPwr*((Kp * errX) + (Kd * dvErrX) + (Ki * iErrX));
           BRMotorPwr = -maxMtrPwr*((Kp * errX) + (Kd * dvErrX) + (Ki * iErrX));

           telemetry.addData("dTime", dTime);
           telemetry.addData("X: ", X);
           telemetry.addData("Y: ", Y);
           //telemetry.addData("Z: ", Z);
           telemetry.addData("MotorPwr: ",FLMotorPwr);
           //telemetry.addData("Xangle: ", Xangle);
           //telemetry.addData("Yangle: ", Yangle);
           telemetry.addData("Zangle: ", Zangle);
           telemetry.update();

           if (java.lang.Math.abs(errX) > 37)
           {
             FLDrive.setPower(FLMotorPwr);
             FRDrive.setPower(FRMotorPwr);
             BLDrive.setPower(BLMotorPwr);
             BRDrive.setPower(BRMotorPwr);
           }
           else
           {if (psi <= -0.5 || psi >= 0.5 && autoStage < 1)
             {
             AngleCorrection(psi, FLDrive, FRDrive, BLDrive, BRDrive);
             }
 else if (autoStage < 1)
             {
               autoStage = 1;
             }
             if (autoStage == 1)
             {
               if (Y < -886.3 && Y > -890.3)
               {
                 FLDrive.setPower(0);
                FRDrive.setPower(0);
                BLDrive.setPower(0);
                BRDrive.setPower(0);
                 autoStage = 2;
               }
               else if (Y > -888.3)
               {
                FLDrive.setPower(0.25);
                FRDrive.setPower(-0.25);
                BLDrive.setPower(-0.25);
                BRDrive.setPower(0.25);
               }
               else if (Y < -888.3)
               {
                 FLDrive.setPower(-0.25);
                FRDrive.setPower(0.25);
                BLDrive.setPower(0.25);
                BRDrive.setPower(-0.25);
               }
             }
             if (autoStage == 2)
             {
             if (psi <= -1 || psi >= 1)
             {
             AngleCorrection(psi, FLDrive, FRDrive, BLDrive, BRDrive);
             }
             else
             {
               autoStage = 3;
             }
             }
           }
           // What type of VuMark is it?
           if (vuMarkResult.name.equals("Front Wall Target")) {
             telemetry.addData("Relic Target", "Front Wall Target");
           } else if (vuMarkResult.name.equals("Red Tower Goal Target")) {
             telemetry.addData("Relic Target", "Red Tower Goal Target");
           } else if (vuMarkResult.name.equals("Red Alliance Target")) {
             telemetry.addData("Target", "Red Alliance Target");
           } else {
             telemetry.addData("Relic Target", "VuMark of UNKNOWN type...");
           }
         } else {
           // No, we don't see one.
           telemetry.addData("VuMark", "No VuMarks are visible.");
           //leftDrive.setPower(0.2);
           //rightDrive.setPower(-0.2);
           //sleep(200);
           //leftDrive.setPower(0);
           //rightDrive.setPower(0);

         }
         telemetry.update();
       }
       // Deactivate before exiting.
       vuforiaUltimateGoal.deactivate();
     }

     vuforiaUltimateGoal.close();
   }
  public void AngleCorrection(double psi, DcMotor FLDrive, DcMotor FRDrive, DcMotor BLDrive, DcMotor BRDrive)
   {
   FLDrive.setPower(0);
   FRDrive.setPower(0);
   BLDrive.setPower(0);
   BRDrive.setPower(0);
   if (psi >= -1 && psi <= 1)
   {
   FLDrive.setPower(0);
   FRDrive.setPower(0);
   BLDrive.setPower(0);
   BRDrive.setPower(0);
             }
             else if (psi > 1)
             {
             FLDrive.setPower(0.15);
             FRDrive.setPower(-0.15);
             BLDrive.setPower(0.15);
             BRDrive.setPower(-0.15);
             }
             else if (psi < -1)
             {
             FLDrive.setPower(-0.15);
             BLDrive.setPower(-0.15);
             FRDrive.setPower(0.15);
             BRDrive.setPower(0.15);
             }
   }
 }
