// package org.firstinspires.ftc.teamcode.subassemblies;
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.RobotLog;
// 
// import org.firstinspires.ftc.teamcode.util.Names;
// //import org.firstinspires.ftc.teamcode.util.RobotReport;
// 
// import java.util.Hashtable;
// 
// /**
//  * Motors - 2x 1620
//  */
// public class Shooter  {
// 
//     public static final double rangeTolerance = 100; //TODO: Set this value! Millimeters distance from target tolerance +-.
//     public static final double rangeAtDefaultAngle = 2000; //TODO: Set this value! Millimeters distance from target at default pitch of 30 degrees.
//     public static final double headingTolerance = 10; //TODO: Set this value! Degrees of heading tolerance +-.
//     public static final double pitchTolerance = 10; //TODO: Set this value! Degrees tolerance +-.
//     public static final double firingRPM = 4800;
//     public static final double motorEncoderEventsPerRotation = 753.2;
// 
// 
//     private DcMotor shooterMotor1 = null;
//     private DcMotor shooterMotor2 = null;
// 
//     private Servo magazine = null;
//     private Servo trigger = null;
//     float power = 0.95f;
// 
//     // I don't know what these positions should be; trial and error will determine these values.
//     private Hashtable<String, Double> triggerPosition  = new Hashtable<String, Double>(){{
//         put("RESET", (double) 0.20);
//         put("SHOOT", (double) 0);
//         put("magazine-down", (double) 0);
//         put("magazine-up", (double) 0.3);
//     }};
// 
//     /**
//      * Pass in the hardware map in the constructor in order to get the motor.
//      * @param hardwareMap pass in the hardware map from the OpMode.
//      */
//     public Shooter(HardwareMap hardwareMap) {
// 
//         shooterMotor1  = hardwareMap.get(DcMotor.class, Names.motor_Shooter1);
//         shooterMotor2 = hardwareMap.get(DcMotor.class, Names.motor_Shooter2);
//         magazine = hardwareMap.get(Servo.class, Names.servo_Magazine);
//         trigger = hardwareMap.get(Servo.class, Names.servo_Trigger);
// 
//         shooterMotor1.setDirection(DcMotor.Direction.REVERSE);
//         shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
// 
//         // Do we intend to use the encoders to measure our speed?
//         shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 
//         ((DcMotorEx)shooterMotor1).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//         ((DcMotorEx)shooterMotor2).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
// 
// 
//         //trigger = hardwareMap.servo.get("trigger");
//         // initialize to reasonable values here.
// 
//         RobotLog.i("SHOOTER Initialized.");
//     }
// 
//     /**
//      * Turn the motors on, get the flywheel spinning.
//      */
//     public void prepForShooting() {
//         shooterMotor1.setPower(power);
//         shooterMotor2.setPower(power);
//         magazine.setPosition(triggerPosition.get("magazine-up"));
//         //log("Shooting");
// 
//         // Show the elapsed game time and wheel power.
//         // RobotLog.i("SHOOTER Status: Run Time: " + runtime.toString());
//         RobotLog.i("SHOOTER Motors: 1 (%.2f), 2 (%.2f)", power, power);
//     }
// 
//     /**
//      * If the power setting on both motors is within tolerance of our expected power, we're good to go.
//      * Is there a better way to measure this?  Do we need to?
//      * If we run with encoders, how does that affect the potential flywheel speed?
//      * @return
//      */
//     public boolean isUpToSpeed() {
//         // a little wiggle room on the power setting
//         //if ((shooterMotor1.getPower() > (power - 0.1)) && (shooterMotor2.getPower() > (power - 0.1))) {
//         //    return true;
//         //}
// 
//         try {
//             // measure RPMs using encoders.
//             ElapsedTime time = new ElapsedTime();
//             time.reset();
//             int startPos1 = shooterMotor1.getCurrentPosition();
//             Thread.sleep(500);
//             float ms = (float)time.milliseconds();
//             int endPos1 = shooterMotor1.getCurrentPosition();
// 
//             float deltaPos1 = endPos1 - startPos1;
//             float rotations1 = deltaPos1/(float)motorEncoderEventsPerRotation; // how many rotations
//             float rpm1 = (rotations1/ms) * 60000; // rotations per minute.
//             if (rpm1 >= firingRPM) { return true; }
// 
//         } catch (InterruptedException ex) {
//             RobotLog.logStackTrace(ex);
//         }
// 
//         return false;
//     }
// 
//     /**
//      * Get measured speed by encoders.
//      * @return RPM
//      */
//     public float getSpeed() {
//         try {
//             // measure RPMs using encoders.
//             ElapsedTime time = new ElapsedTime();
//             time.reset();
//             int startPos1 = shooterMotor1.getCurrentPosition();
//             Thread.sleep(500);
//             float ms = (float)time.milliseconds();
//             int endPos1 = shooterMotor1.getCurrentPosition();
// 
//             float deltaPos1 = endPos1 - startPos1;
//             float rotations1 = deltaPos1/(float)motorEncoderEventsPerRotation; // how many rotations
//             float rpm1 = (rotations1/ms) * 60000; // rotations per minute.
//             return rpm1;
// 
//         } catch (InterruptedException ex) {
//             RobotLog.logStackTrace(ex);
//         }
//         return 0;
//     }
// 
//     /**
//      * Self-explanatory.
//      * I know that this is a horrible documentation comment, but it _is_ self-explanatory.
//      */
//     public void shoot() {
//         RobotLog.i("Shooting ring.");
//         try {
//             trigger.setPosition(triggerPosition.get("SHOOT"));
//             Thread.sleep(500);
//             trigger.setPosition(triggerPosition.get("RESET"));
//  //           RobotReport.ringsOnBot--;
//         } catch(InterruptedException e) {};
//     }
// 
//     /**
//      * Stops the shooter motors.
//      */
//     public void stop() {
//         RobotLog.i("Stopping the shooter motor.");
//         shooterMotor1.setPower(0);
//         shooterMotor2.setPower(0);
//         magazine.setPosition(triggerPosition.get("magazine-down"));
//     }
// 
//     /**
//      * Detect the number of rings in the holding area.
//      * @return The number of rings
//      */
//     public int detectRings() {
//         return 0;
//     }
// }
// 