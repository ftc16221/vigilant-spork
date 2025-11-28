// package org.firstinspires.ftc.teamcode.subassemblies;
// 
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.hardware.Servo;
// 
// /**
//  * Grabber must mount on the _right hand rear_ corner
//  * Grabber must mount __ inches high.
//  * 312 rpm motor
//  * servo
//  *
//  */
// public class WobbleGoalGrabber {
// 
//     public WobbleGoalGrabber(HardwareMap hardwareMap) {
//         wggServo = hardwareMap.servo.get("arm");
//         grabberServo = hardwareMap.servo.get("claw");
//     }
// 
//     public Servo wggServo;
//     public Servo grabberServo;
// 
//     public void down() {
//         wggServo.setPosition(0.2);
//     }
//     public void up() {
//         wggServo.setPosition(0.55);
//     }
//     public void rest() {wggServo.setPosition(0.75);}
//     public void grab() {
//       grabberServo.setPosition(0.8);
//     }
//     public void release() {
//       grabberServo.setPosition(0.6);
//     }
//     /**
//     * Down, grab, up.
//     */
//     public void grabWrapper() {
//       down();
//       grab();
//       up();
//     }
//     /**
//     * Down, release, up.
//     */
//     public void releaseWrapper() {
//       down();
//       release();
//       up();
//     }
// }
// 