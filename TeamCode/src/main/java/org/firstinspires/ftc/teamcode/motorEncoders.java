// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.robotcore.external.navigation.Position;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// 
// @Autonomous(name = "motorEncoders")
// public class motorEncoders extends LinearOpMode {
// 
//   private DcMotor left_front;
//   private DcMotor right_front;
//   private DcMotor left_rear;
//   private DcMotor right_rear;
// 
//   /**
//    * This function is executed when this Op Mode is selected from the Driver Station.
//    */
//   @Override
//   public void runOpMode() {
//     left_front = hardwareMap.get(DcMotor.class, "left_front");
//     right_front = hardwareMap.get(DcMotor.class, "right_front");
//     left_rear = hardwareMap.get(DcMotor.class, "left_rear");
//     right_rear = hardwareMap.get(DcMotor.class, "right_rear");
// 
//     // Reverse direction of the left side drive motors
//     left_front.setDirection(DcMotorSimple.Direction.REVERSE);
//     right_front.setDirection(DcMotorSimple.Direction.FORWARD);
//     left_rear.setDirection(DcMotorSimple.Direction.REVERSE);
//     right_rear.setDirection(DcMotorSimple.Direction.FORWARD);
//     
//     // Set Zero Power Behavior of all four drive motors
//     left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//     right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//     left_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//     right_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//     
//     waitForStart();
//     
//     if (opModeIsActive()) {
//       
//       // Reset motor encoder values before beginning movement
//       left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       
//       while (opModeIsActive()) {
// 
//         // set motor powers (speed)
//         left_front.setPower(0.25);
//         right_front.setPower(0.25);
//         left_rear.setPower(0.25);
//         right_rear.setPower(0.25);
//         
//         // set target position
//         left_front.setTargetPosition(4301);
//         right_front.setTargetPosition(4301);
//         left_rear.setTargetPosition(4301);
//         right_rear.setTargetPosition(4301);
//         
//         // set position tolerannce
//         ((DcMotorEx) left_front).setTargetPositionTolerance(0);
//         ((DcMotorEx) right_front).setTargetPositionTolerance(0);
//         ((DcMotorEx) left_rear).setTargetPositionTolerance(0);
//         ((DcMotorEx) right_rear).setTargetPositionTolerance(0);
//     
//         
//         // run motors to set position at set speed
//         while (!(((DcMotorEx) left_front).isBusy() && ((DcMotorEx) right_front).isBusy() && ((DcMotorEx) left_rear).isBusy() && ((DcMotorEx) right_rear).isBusy())) 
//         {
//           left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//           right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//           left_rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//           right_rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         }
//         
//         /* stop motors and reset encoders at end
//         left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         */
//         telemetry.addData("encoder LF", left_front.getCurrentPosition());
//         telemetry.addData("encoder RF", right_front.getCurrentPosition());
//         telemetry.addData("encoder LR", left_rear.getCurrentPosition());
//         telemetry.addData("encoder RR", right_rear.getCurrentPosition());
//         telemetry.update();
//       }
//     }
//   }
// }
// 