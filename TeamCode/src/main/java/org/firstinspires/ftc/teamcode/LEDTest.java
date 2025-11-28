// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// 
// @TeleOp(name = "LEDTest (Blocks to Java)")
// public class LEDTest extends LinearOpMode {
// 
//   private RevBlinkinLedDriver readyLED;
// 
//   /**
//    * This function is executed when this Op Mode is selected from the Driver Station.
//    */
//   @Override
//   public void runOpMode() {
//     double LEDvalue;
//     double LEDdirection;
// 
//     readyLED = hardwareMap.get(RevBlinkinLedDriver.class, "readyLED");
// 
//     // Set servo to mid position
//     LEDvalue = 0.1;
//     LEDdirection = 1;
//     readyLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//     waitForStart();
//     while (opModeIsActive()) {
//       // Keep in valid range
//       if (LEDvalue >= 0.5) {
//         readyLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
//         if (LEDvalue == 0.6) {
//           readyLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//           LEDdirection = -1;
//         }
//       }
//       if (LEDvalue < 0.5) {
//         readyLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK);
//         LEDdirection = 1;
//       }
//       telemetry.addData("LED", LEDvalue);
//       telemetry.update();
//       sleep(1000);
//       LEDvalue = Math.round(100 * (LEDvalue + 0.1 * LEDdirection));
//       LEDvalue = LEDvalue / 100;
//     }
//   }
// }
// 