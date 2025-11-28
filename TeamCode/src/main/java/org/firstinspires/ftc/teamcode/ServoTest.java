 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.Servo;

 @TeleOp(name = "ServoTest (Blocks to Java)")
 public class ServoTest extends LinearOpMode {

   private Servo arm;
   private Servo claw;

   /**
    * This function is executed when this Op Mode is selected from the Driver Station.
    */
   @Override
   public void runOpMode() {
     double ServoPosition;
     double ServoDirection;
     double ServoSpeed;

     arm = hardwareMap.get(Servo.class, "arm");
     claw = hardwareMap.get(Servo.class, "claw");

     // Set servo to mid position
     ServoPosition = 0.18;
     ServoDirection = 1;
     ServoSpeed = 0.01;
     arm.setPosition(0.18);
     claw.setPosition(0.6);
     waitForStart();
     while (opModeIsActive()) {
       // Use gamepad X and B to open close servo
       claw.setPosition(0.8);
       sleep(1000);
       // Keep Servo position in valid range
       if (ServoPosition == 0.58) {
         ServoDirection = -1;
       }
       if (ServoPosition == 0.18) {
         ServoDirection = 1;
       }
       arm.setPosition(0.55);
       telemetry.addData("Servo", ServoPosition);
       telemetry.update();
       sleep(2000);
       arm.setPosition(0.2);
       sleep(1500);
       ServoPosition = Math.round(100 * (ServoPosition + 0.1 * ServoDirection));
       ServoPosition = ServoPosition / 100;
     }
   }
 }
