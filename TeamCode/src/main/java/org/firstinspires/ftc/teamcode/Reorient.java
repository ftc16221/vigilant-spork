package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.List;



//***************************************************************************************************************************
@Autonomous(name = "Reorient")

public class Reorient extends LinearOpMode {
  
  private VuforiaCurrentGame vuforiaUltimateGoal;
  private Blinker expansion_Hub_1;
  private Blinker expansion_Hub_2;
  private DistanceSensor frontIR;
  private DistanceSensor rearIR;
  private DistanceSensor leftIR;
  private DistanceSensor rightIR;
  private ColorSensor color_sensor;
  private DcMotor FLDrive;
  private DcMotor FRDrive;
  private DcMotor BLDrive;
  private DcMotor BRDrive;
  private BNO055IMU imu;

//***************************************************************************************************************************
  public class Orientation {
    
    // declare variables
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
    
    //constructor
    public Orientation(){
      
    }    
    //methods
    void update(){
      quat = imu.getQuaternionOrientation();
      q0 = quat.w;
      q1 = quat.x;
      q2 = quat.y;
      q3 = quat.z;
      //
      // convert quaternion to Euler angles.
      // psi = yaw (or heading)
      psi = Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) / Math.PI * 180;
      // theta = pitch
      theta = Math.asin(2 * (q0 * q2 - q3 * q1)) / Math.PI * 180;
      // phi = roll
      phi = Math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) / Math.PI * 180;
      // store gravity vector from IMU
      gravity = imu.getGravity();
    }
    double getPsi(){
      update();
      return psi; //yaw (heading)
    }
    double getTheta(){
      update();
      return theta; //pitch
    }
    double getPhi(){
      update();
      return phi; //roll
    }
    double getGx(){
      return gravity.xAccel;
    }
    double getGy(){
      return gravity.yAccel;
    }
    double getGz(){
      return gravity.zAccel;
    }
  } //END:   class Orientation

//***************************************************************************************************************************  
//
//  This class determines the bot's field position using 2 to 4 IR sensors,
//  but it is limited by the sensor's max range (1.2m) versus 
//  the field size (nearest wall can be up to 1.79m away from a sensor).
//  On an 8' x 12' field, it will accurately find the bot's location as long
//  as the bot is within 4' of the North or South walls, but not in the region
//  from that boundary to the mid field.   It will accurately find the location
//  along the Y axis (East/West) for an 8' wide field.
//
//  For a traditional 12' x 12' dimensioned field, it will not find position
//  except within 4' distances of the 4 field corners.
//

  public class BotPosition{   
    //declare all variables
      public String Name;
      private double[] Actual;
      double distRight; // raw distance measured by sensor, in mm
      double distLeft;  // raw distance measured by sensor, in mm
      double distFront; // raw distance measured by sensor, in mm
      double distRear;  // raw distance measured by sensor, in mm
      double offsetLeft = 154.3; //offset in mm from center of bot, in viewing direction of sensor
      double offsetRight = 159.4; //offset in mm from center of bot, in viewing direction of sensor
      double offsetFront = 161.8; //offset in mm from center of bot, in viewing direction of sensor
      double offsetRear = 209.55; //offset in mm from center of bot, in viewing direction of sensor
      boolean statusLeft;  // sensor status, must be within maxIRrange to be true
      boolean statusRight; // sensor status, must be within maxIRrange to be true
      boolean statusFront; // sensor status, must be within maxIRrange to be true
      boolean statusRear;  // sensor status, must be within maxIRrange to be true
      boolean statusIRx; // composite status of X direction sensors
      boolean statusIRy; // composite status of Y direction sensors
      double IRx;  // position on field, X coordinate
      double IRx_sum; // for rolling average
      double IRy;  // position on field, Y coordinate
      double IRy_sum; // used for rolling average
      double countX; //count of valid X measurements during rolling average window
      double countY; //count of valid Y measurements during rolling average window
      double maxIRrange = 1200; // max range in mm of REV 2M Distance Sensor (uses ST Micro VL53L0X)

    //constructor
    BotPosition(){        
        Actual = new double[2];
//      Name = name;
    } // END:  BotPosition constructor
    
    //methods
    void update(){
        //rotate to 90 degrees
        
        IRx_sum = 0;
        IRy_sum = 0;
        countY = 0;
        countX = 0;

      for(int i = 0; i < 5; i++){ // NOTE:  # loops = # of samples in rolling average, and loop time will be # samples * 33ms/sample
        //reset status of all IR sensors to false on each pass
        statusRight = false;
        statusLeft = false;
        statusFront = false;
        statusRear = false;
        statusIRx = false;
        statusIRy = false;
        
        //get distance from each IR sensor (bot at 90 degrees), and set status to true on each measuing within range
        distRight = rightIR.getDistance(DistanceUnit.MM);
        if(distRight <= maxIRrange){
                statusRight = true;
            }         
        distLeft = leftIR.getDistance(DistanceUnit.MM);
        if(distLeft <= maxIRrange){
                statusLeft = true;
            }     
        distFront = frontIR.getDistance(DistanceUnit.MM);
        if(distFront <= maxIRrange){
                statusFront = true;
            }    
        distRear = rearIR.getDistance(DistanceUnit.MM);
        if(distRear <= maxIRrange){
                statusRear = true;
            }             
        
        //if VuMark is visible, get current position
        
        //calculate current position
            //if within 2 feet of visible VuMark, use VuMark position, otherwise use IR sensors

        // calculate IRx and IRy based on status of sensors on opposite sides
        if(statusRight==true && statusLeft==false){
            IRy = -(1790 - distRight - offsetRight);
            statusIRy = true;
        } else if (statusRight==false && statusLeft==true){
            IRy = 598 - distLeft - offsetLeft;
            statusIRy = true;
        } else if (statusRight==true && statusLeft==true){
            IRy = ((598 - distLeft - offsetLeft)+(-(1790-distRight-offsetRight)))/2;
            statusIRy = true;
        } else {
            IRy = 0;
            statusIRy = false;
        }
        
        if(statusIRy){
            IRy_sum = IRy_sum + IRy;
            countY += 1; 
        } // END: if(statusIRy)
        
        if(statusFront==true && statusRear==false){
            IRx = (1790 - distFront - offsetFront);
            statusIRx = true;
        } else if (statusFront==false && statusRear==true){
            IRx = -(1790-distRear-offsetRear);
            statusIRx = true;
        } else if (statusFront==true && statusRear==true){
            IRx = ((1790 - distFront - offsetFront)+(-(1790-distRear-offsetRear)))/2;
            statusIRx = true;
        } else {
            IRx = 0;
            statusIRx = false;
        }

        if(statusIRx){
            IRx_sum = IRx_sum + IRx;
            countX +=1;
        } // END:  if(statusIRx)
      }  // END:  for loop
      
      IRy = IRy_sum / countY;
      IRx = IRx_sum / countX;
      
        // calculate drift between target position and current position
        
        //store current position and drift, and log both
    }  //END:  update method

    double[] getPosition(){
        update();
        
        /*  ----debugguing section -----
        telemetry.addData("statusLeft: ",statusLeft);
        telemetry.addData("statusRight: ",statusRight);
        telemetry.addData("statusFront: ",statusFront);
        telemetry.addData("statusRear: ",statusRear);
        telemetry.addData("statusIRx: ",statusIRx);
        telemetry.addData("statusIRy: ",statusIRy);
        telemetry.addData("IRx = ",IRx);
        telemetry.addData("IRy = ",IRy);
        telemetry.addData("rangeX = ",rangeX);
        telemetry.addData("rangeY = ",rangeY);
        telemetry.addData("distLeft = ",distLeft/25.4);
        telemetry.addData("distRight = ",distRight/25.4);
        telemetry.addData("distFront = ",distFront/25.4);
        telemetry.addData("distRear = ",distRear/25.4);
        telemetry.update();
        ----- debugging section ----------  */

        Actual[0] = IRx;
        Actual[1] = IRy;
        return Actual;
        
    } // END:  method getPosition
    
      
  } //END:  class BotPosition
  
//**********************************************************************

  public class myPath{
    //declare variables
      double originX = 0;
      double originY = 0;
      double targetX = 0;
      double targetY = 0;
      double initHeading = 0;
      double newHeading;
      double oppHeading;
      String originName,targetName,s;
      double deltaX;
      double deltaY;
      double heading;
      double finalHeading;
      double leastRot;
      double direction;
      double distance;
      double rot_L_fwd,rot_R_fwd,rot_L_rev,rot_R_rev;
    
    //constructor
    public myPath()
    {
      double originX;
      double originY;
      double targetX;
      double targetY;
      double initHeading;
      double newHeading;
      double oppHeading;
      String originName,targetName;
      double deltaX;
      double deltaY;
      double heading;
      double finalHeading;
      double leastRot;
      double direction;
      double distance;
      double rot_L_fwd,rot_R_fwd,rot_L_rev,rot_R_rev;
  
    }
    
    //methods
    double calcMyPath(double originX,double originY,double targetX,double targetY,double initHeading){

      deltaX = targetX - originX;
      deltaY = targetY - originY;
      newHeading = Math.atan2(deltaY,deltaX);
      newHeading = java.lang.Math.toDegrees(newHeading) + 90;
      if(newHeading < 180){
        oppHeading = newHeading + 180;
      }
      else {
        oppHeading = newHeading - 180;
      }
      

      distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
      rot_L_fwd = (360 - initHeading + newHeading) % 360;
      rot_R_fwd = -(360 - newHeading + initHeading) % 360;
      rot_L_rev = (360 - initHeading + oppHeading) % 360;
      rot_R_rev = -(360 - oppHeading + initHeading) % 360;
      leastRot = rot_L_fwd;
      direction = 1;
      if (Math.abs(rot_R_fwd)<Math.abs(leastRot)){
        leastRot = rot_R_fwd;
      }
      if(Math.abs(rot_L_rev)<Math.abs(leastRot)){
        leastRot = rot_L_rev;
        direction = -1;
      }
      if (Math.abs(rot_R_rev)<Math.abs(leastRot)){
        leastRot = rot_R_rev;
        direction = -1;
      }
      finalHeading = initHeading + leastRot;
      return 1;
    }
    double getHeading(double originX,double originY,double targetX,double targetY,double initHeading){
      calcMyPath(originX,originY,targetX,targetY,initHeading);
      return finalHeading;
    }
    double getDistance(double originX,double originY,double targetX,double targetY,double initHeading){
      calcMyPath(originX,originY,targetX,targetY,initHeading);
      return distance;
    }
    double getRotation(double originX,double originY,double targetX,double targetY,double initHeading){
      calcMyPath(originX,originY,targetX,targetY,initHeading);
      return leastRot;
    }
    double getDirection(double originX,double originY,double targetX,double targetY,double initHeading){
      calcMyPath(originX,originY,targetX,targetY,initHeading);
      return direction;
    }
  } // END:  class myPath

//********************************************************************************************************************************
  public class Destination {
      
      //declare variables
      public String destName;
      private float[] Coordinate;
      
      //constructor
      public Destination(String name,float destX,float destY){
        Coordinate = new float[2];
        destName = name;
        Coordinate[0] = destX;
        Coordinate[1] = destY;
      }
      
      //methods
      String getName(){
        return destName;
      }
      float getX(){
        return Coordinate[0];
      }
      float getY(){
        return Coordinate[1];
      }
  } //END:  class Destination

//*************************************************************************************************************************
  public void runOpMode() {    
    
    //declare variables for quaternion math using BNO055 IMU
    BNO055IMU.Parameters imuParameters;

//------------------------------------------------------------------------------------
    //map to IMU hardware
    imu = hardwareMap.get(BNO055IMU.class, "imu");
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
    
//------------------------------------------------------------------------------------    
    //declare variables for PID control system (proportional-integral-derivative)
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
    double IRerrX = 0;   // used for IR distance sensor, X direction
    double IRerrY = 0;   // used for IR distance sensor, Y direction
    //********************PID COEFFICIENTS***********************
    double Kp = 0.000833;
    double Kd = -0.0006244;
    double Ki = -0.0000888;
    //***********************************************************
    
//------------------------------------------------------------------------------------
    //declare variables for timers
    double msTime_now = 0;
    double msTime_prev = 0;
    double dTime = 0;
    // create timer object
    ElapsedTime msTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

//------------------------------------------------------------------------------------
//map hardware and declare variables for IR distance sensors
    
    rightIR = hardwareMap.get(DistanceSensor.class, "right");
    leftIR = hardwareMap.get(DistanceSensor.class, "left");
    frontIR = hardwareMap.get(DistanceSensor.class, "front");
    rearIR = hardwareMap.get(DistanceSensor.class, "rear");
    double leftWall = 0;
    double rightWall = 0;
    double frontWall = 0;
    double backWall = 0;
    double distanceWall = 0;

//------------------------------------------------------------------------------------    
    //create Destination objects
    Destination SHO = new Destination("Shoot",100,-901);
    Destination STS = new Destination("starterStack",560,-901);
    Destination TZA = new Destination("TargetZoneA",307,-1501);
    Destination TZB = new Destination("TargetZoneB",904,-904);
    Destination TZC = new Destination("TargetZoneC",1501,-1501);
    Destination SLC = new Destination("StartLineLeftCenter",-1501,-622);
    Destination SRC = new Destination("StartLineRightCenter",-1501,-1238);
    Destination SLT = new Destination("StartLineLeftTip",-1212,-627);
    Destination SRT = new Destination("StartLineRightTip",-1212,-1238);
    Destination TOW = new Destination("TowerGoal",1790,-888);
    Destination PSL = new Destination("PS_LEFT",1790,-107);
    Destination PSC = new Destination("PS_CENTER",1790,-298);
    Destination PSR = new Destination("PS_RIGHT",1790,-488);  
  
//------------------------------------------------------------------------------------  
    // declare variables for motor powers on mecanum drive base
    double FLMotorPwr = 0;
    double FRMotorPwr = 0;
    double BLMotorPwr = 0;
    double BRMotorPwr = 0;
    double maxMtrPwr = 1;
    // map hardware to variables for mecanum drive motors
    FLDrive  = hardwareMap.dcMotor.get("left_front");
    FRDrive  = hardwareMap.dcMotor.get("right_front");
    BLDrive  = hardwareMap.dcMotor.get("left_rear");
    BRDrive  = hardwareMap.dcMotor.get("right_rear");
    // Set all motors to zero power
    FLDrive.setPower(0);
    FRDrive.setPower(0);
    BLDrive.setPower(0);
    BRDrive.setPower(0);
    // set motor directions for mecanum drive base
    FLDrive.setDirection(DcMotor.Direction.REVERSE);
    FRDrive.setDirection(DcMotor.Direction.FORWARD);
    BLDrive.setDirection(DcMotor.Direction.REVERSE);
    BRDrive.setDirection(DcMotor.Direction.FORWARD);
    // Set Zero Power Behavior of all four drive motors
    FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//------------------------------------------------------------------------------------    
    //declare variables for VuMark result tracking using VuForia
    VuforiaBase.TrackingResults vuMarkResult;
    float X;
    float Y;
    float Z;
    float Xangle;
    float Yangle;
    float Zangle;
    int autoStage = 0;
    // create Vuforia object
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

//------------------------------------------------------------------------------------    
    //declare & initialize variables for using myPath class
    double startX = 0;
    double startY = 0;
    double endX = 0;
    double endY = 0;
    double initialHeading = 0;
    double currentHeading = 0;
    double finalHeading = 0;
    double initialPitch = 0;
    double targetPitch = 0;
    double targetHeading = 0;
    double targetDistance = 0;
    double targetRotation = 0;
    double errHeading = 0;
    double errDistance = 0;
    double errPitch = 0;
    double targetDirection = 0;
    int stage = 0;
    String s; //text string for logging
    
//----------------------------------------------------------

    double[] IRcoordinate; // used for returning values from BotPosition methods

//------------------------------------------------------------------------------------    
    // Prompt user to push start button.
    telemetry.addData("Reorient initialized, ", "Press start to continue...");
    telemetry.update();
    // Wait until user pushes start button.
    waitForStart();
    
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    stage = 0;
    msTime.reset();
    
    if (opModeIsActive()) {
      
      // Activate Vuforia software.
      vuforiaUltimateGoal.activate();
      
      // set initial motor mode
      FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      
      
      Orientation Pose = new Orientation();
      
      while (opModeIsActive()) {

    if(stage==0){
        BotPosition here = new BotPosition();
        IRcoordinate = here.getPosition();
        
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
          telemetry.addData("VuMark X = ",X);
          telemetry.addData("VuMark Y = ",Y);
          

        } else {
          telemetry.addData("VuMark","No VuMark is visible....");
        }
        
        telemetry.addData("IR X = ",IRcoordinate[0]);
        telemetry.addData("IR Y = ",IRcoordinate[1]);
        telemetry.update();
        
    }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
      // calcMyPath
      if(stage==1){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath start: ",s);  

        initialHeading = Pose.getPsi()+90;// +90 points North
        initialPitch = 25; //PLACEHOLDER
        startX = SLC.getX();
        startY = SLC.getY();
        endX = TZB.getX();
        endY = TZB.getY();
      
        myPath path = new myPath();
        path.calcMyPath(startX,startY,endX,endY,initialHeading);
        targetHeading = path.getHeading(startX,startY,endX,endY,initialHeading);
        targetDistance =path.getDistance(startX,startY,endX,endY,initialHeading);
        targetRotation = path.getRotation(startX,startY,endX,endY,initialHeading);
        targetDirection = path.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting

        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath complete: ",s);
        
        stage += 1;
      } // End:  calcMyPath

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      //rotate
      if(stage==2){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   rotate start: ",s);
        
        currentHeading = Pose.getPsi()+90; // + 90 points North
        errHeading = targetHeading - currentHeading;
        
        telemetry.addData("startX: ",startX);
        s = Double.toString(startX);      
        Log.i("***16221_LOG***   startX: ",s);    
        s = Double.toString(startY);      
        Log.i("***16221_LOG***   startY: ",s);
        telemetry.addData("endX: ",endX);
        telemetry.addData("initialHeading: ",initialHeading);
        telemetry.addData("currentHeading: ", currentHeading);
        s = Double.toString(currentHeading);      
        Log.i("***16221_LOG***   currentHeading: ",s);  
        telemetry.addData("targetHeading: ",targetHeading);
        s = Double.toString(targetHeading);      
        Log.i("***16221_LOG***   targetHeading: ",s);    
        telemetry.addData("errHeading: ",errHeading);
        s = Double.toString(errHeading);      
        Log.i("***16221_LOG***   errHeading: ",s);     
        telemetry.update();
        
        if (errHeading <= -1 || errHeading >= 1) //if not aligned straight at target, errHeading=0, then realign
            {
            s = Double.toString(errHeading);      
            Log.i("***16221_LOG***   errHeadingCorrection: ",s); 
            AngleChange(errHeading, FLDrive, FRDrive, BLDrive, BRDrive);

            }
        if(errHeading >= -1 && errHeading <= 1)
            {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);    
            s = Double.toString(stage);
            Log.i("***16221_LOG***   rotate complete: ",s);
            stage +=1;
            }
      } // End:  rotate

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - motor mode change & encoder reset
      if (stage ==3){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   translate - motor init & reset start: ",s);
        
        // Reset motor encoder values before beginning movement
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
        // Reset motor encoder values before beginning movement
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        s = Double.toString(stage); 
        Log.i("***16221_LOG***  translate - motor init & reset complete ",s); 
        stage += 1;
      } // End:  translate - modor mode change & encoder reset

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - move    
      if (stage == 4){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Translate - move: ",s);
        
        
        move(targetDistance, maxMtrPwr, targetDirection, stage, FLDrive, FRDrive, BLDrive, BRDrive);
        if((!((DcMotorEx) FLDrive).isBusy() && !((DcMotorEx) FRDrive).isBusy() && !((DcMotorEx) BLDrive).isBusy() && !((DcMotorEx) BRDrive).isBusy()))
        {
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Transate - move complete: ",s);
        stage += 1;
        }
      } // End:  translate - move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // translate - re-align post-move
      if (stage == 5){
        s = Double.toString(stage);         
        Log.i("***16221_LOG***   re-align post-move begin: ",s);
        //
        s = Double.toString(stage);
        Log.i("***16221_LOG***   re-align post-move complete: ",s);
        stage += 1;
      } // End: translate - re-align post-move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
      // aim to shoot
      if(stage==6){
        s=Double.toString(stage); 
        Log.i("***16221_LOG***   AIM stage start: ",s);
        
        initialHeading = Pose.getPsi()+90;// +90 points North
        initialPitch = 25; //PLACEHOLDER
        startX = TZA.getX();
        startY = TZA.getY();
        endX = TOW.getX();
        endY = TOW.getY();
      
        myPath aim = new myPath();
        aim.calcMyPath(startX,startY,endX,endY,initialHeading);
        targetHeading = aim.getHeading(startX,startY,endX,endY,initialHeading);
        targetDistance =aim.getDistance(startX,startY,endX,endY,initialHeading);
        targetRotation = aim.getRotation(startX,startY,endX,endY,initialHeading);
        targetDirection = aim.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting
        if(targetDirection == -1){
          targetHeading = targetHeading - 180;
        }
        targetPitch = 33; //PLACEHOLDER VALUE
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   Aim to shoot complete: ",s);
        stage += 1;
      } //END: aim to shoot

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
      // rotate to shoot
      if(stage == 7){
        s=Double.toString(stage); 
        Log.i("***16221_LOG***   Rotate stage start: ",s);
        
        currentHeading = Pose.getPsi()+90; // + 90 points North
        errHeading = targetHeading - currentHeading;

        s = Double.toString(startX);      
        Log.i("***16221_LOG***   startX: ",s);    
        s = Double.toString(startY);      
        Log.i("***16221_LOG***   startY: ",s);
        s = Double.toString(currentHeading);      
        Log.i("***16221_LOG***   currentHeading: ",s);  
        s = Double.toString(targetHeading);      
        Log.i("***16221_LOG***   targetHeading: ",s);    
        s = Double.toString(errHeading);      
        Log.i("***16221_LOG***   errHeading: ",s);     
        telemetry.update();

        if (errHeading <= -1 || errHeading >= 1) //if not aligned straight at target, errHeading=0, then realign
          {
            AngleChange(errHeading, FLDrive, FRDrive, BLDrive, BRDrive);
          }
        if(errHeading >= -1 && errHeading <= 1)
          {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);  
            s = Double.toString(stage);
            Log.i("***16221_LOG***   Rotate to shoot complete: ",s);
            stage += 1;
          } 
        }  //END:  rotate to shoot
          
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      //pitch to shoot
      if(stage == 8){
        s=Double.toString(stage); 
        Log.i("***16221_LOG***   Pitch stage start: ",s);
        
        errPitch = targetPitch - initialPitch;
        //
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Pitch to shoot complete: ",s);
        stage += 1;
      } //END:  pitch to shoot

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
      //shoot
      if(stage == 9){
          s=Double.toString(stage); 
          Log.i("***16221_LOG***  Shoot stage start: ",s);
      
          //
          s = Double.toString(stage);
          Log.i("***16221_LOG***   Shoot complete: ",s);
          stage += 1;
      } //END: shoot
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    } //END:   While(OpModeIsActive)
  } //END:  If(OpModeIsActive)
} //END:  RunOpMode

//********************************************************************************************************************
  public void AngleCorrection(double psi, DcMotor FLDrive, DcMotor FRDrive, DcMotor BLDrive, DcMotor BRDrive)
  {
      FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    FLDrive.setPower(0);
//    FRDrive.setPower(0);
//    BLDrive.setPower(0);
//    BRDrive.setPower(0);
    if (psi >= -0.5 && psi <= 0.5)
    {
      FLDrive.setPower(0);
      FRDrive.setPower(0);
      BLDrive.setPower(0);
      BRDrive.setPower(0);
    }
    else if (psi > 0.5) //rotate clockwise
    {
      FLDrive.setPower(0.15);
      FRDrive.setPower(-0.15);
      BLDrive.setPower(0.15);
      BRDrive.setPower(-0.15);
    }
    else if (psi < -0.5) //rotate counterclockwise
    {
      FLDrive.setPower(-0.15);
      FRDrive.setPower(0.15);      
      BLDrive.setPower(-0.15);
      BRDrive.setPower(0.15);
    }
  } //END:  AngleCorrection

//********************************************************************************************************************  
  public void AngleChange(double psi, DcMotor FLDrive, DcMotor FRDrive, DcMotor BLDrive, DcMotor BRDrive)
  {   
    FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    if (psi >= -0.5 && psi <= 0.5)
    {
      FLDrive.setPower(0);
      FRDrive.setPower(0);
      BLDrive.setPower(0);
      BRDrive.setPower(0);
    }
    else if (psi > 0.5) //rotate counterclockwise
    {
      FLDrive.setPower(-0.2);
      FRDrive.setPower(0.2);
      BLDrive.setPower(-0.2);
      BRDrive.setPower(0.2);
    }
    else if (psi < -0.5) //rotate clockwise
    {
      FLDrive.setPower(0.2);
      FRDrive.setPower(-0.2);
      BLDrive.setPower(0.2);
      BRDrive.setPower(-0.2);
    }

  } //END:  AngleChange
  
//********************************************************************************************************************  
  public void move(double distance, double speed, double direction, int stage, DcMotor FLDrive, DcMotor FRDrive, DcMotor BLDrive, DcMotor BRDrive){
      double slip;
      double ticksPerMM = 2.3663; // NOTE:  determined this number emperically by running a set # of ticks and dividing by the actual distance traveled
      int targetTicks = (int) Math.round(ticksPerMM * distance);
      int tolerance = 10; // 10 ticks = 4.23 mm traveled (= 0.166 inches)
      String s;
  
      // set motor powers (speed)
      FLDrive.setPower(speed);
      FRDrive.setPower(speed);
      BLDrive.setPower(speed);
      BRDrive.setPower(speed);
      s = Double.toString(speed); 
      Log.i("***16221_LOG***   speed set: ",s); 
 
      // set target position
      FLDrive.setTargetPosition(targetTicks);
      FRDrive.setTargetPosition(targetTicks);
      BLDrive.setTargetPosition(targetTicks);
      BRDrive.setTargetPosition(targetTicks);
      s = Double.toString(targetTicks); 
      Log.i("***16221_LOG***   targetTicks set: ",s);         
        
      // set position tolerannce
      ((DcMotorEx) FLDrive).setTargetPositionTolerance(tolerance);
      ((DcMotorEx) FRDrive).setTargetPositionTolerance(tolerance);
      ((DcMotorEx) BLDrive).setTargetPositionTolerance(tolerance);
      ((DcMotorEx) BRDrive).setTargetPositionTolerance(tolerance);
      s = Double.toString(tolerance); 
      Log.i("***16221_LOG***   tolerance set: ",s); 
      
      if (direction == -1){
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
      } 
      else if (direction == 1){
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
      }
      s = Double.toString(direction); 
      Log.i("***16221_LOG***   direction set: ",s); 


      // turn off each motor as it reaches targetTicks + tolerance, to prevent it 
      // from being pushed beyond tolerance range by other motors that may still
      // active and thus being turned on again during next pass thru the sending loop
      if (FLDrive.getCurrentPosition() >= (targetTicks + tolerance)){
        FLDrive.setPower(0);
      }
      if (FRDrive.getCurrentPosition() >= (targetTicks + tolerance)) {
        FRDrive.setPower(0); 
      }
        if (BLDrive.getCurrentPosition() >= (targetTicks + tolerance)){
        BLDrive.setPower(0);
      }
      if (BRDrive.getCurrentPosition() >= (targetTicks + tolerance)) {
        BRDrive.setPower(0); 
      }
      
      // run motors to set position at set speed
      FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
      s = Double.toString(stage);      
      Log.i("***16221_LOG***   current stage: ",s);
      s = Double.toString(FLDrive.getCurrentPosition());      
      Log.i("***16221_LOG***   encoder FL: ",s);
      s = Double.toString(FRDrive.getCurrentPosition());      
      Log.i("***16221_LOG***   encoder FR: ",s); 
      s = Double.toString(BLDrive.getCurrentPosition());      
      Log.i("***16221_LOG***   encoder BL: ",s); 
      s = Double.toString(BRDrive.getCurrentPosition());      
      Log.i("***16221_LOG***   encoder BR: ",s); 
 
      
      if((!((DcMotorEx) FLDrive).isBusy() && !((DcMotorEx) FRDrive).isBusy() && !((DcMotorEx) BLDrive).isBusy() && !((DcMotorEx) BRDrive).isBusy()))
      {
        // stop motors and reset encoders at end
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   STOP & RESET ENCODERS: ",s); 
      
      }
      return;
  }  // END:  move
//********************************************************************************************************************

}
