// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.OrientationSensor;
// import java.util.ArrayList;
// import java.util.List;
// import org.firstinspires.ftc.robotcore.external.JavaUtil;
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.Axis;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
// 
// @TeleOp(name = "IMU_quaternion (Blocks to Java)", group = "")
// public class IMU_quaternion extends LinearOpMode {
// 
//   private BNO055IMU imu;
// 
//   /**
//    * This function is executed when this Op Mode is selected from the Driver Station.
//    */
//   @Override
//   public void runOpMode() {
//     BNO055IMU.Parameters imuParameters;
//     Quaternion quat;
//     float q0;
//     float q1;
//     float q2;
//     float q3;
//     double psi;
//     double theta;
//     double phi;
//     Orientation angles;
//     List<Axis> foo;
//     Acceleration gravity;
// 
//     imu = hardwareMap.get(BNO055IMU.class, "imu");
// 
//     // Create new IMU Parameters object.
//     imuParameters = new BNO055IMU.Parameters();
//     //  set mode to "IMU". This will use the
//     // device's gyro and its accelerometer
//     // to calculate the relative orientation of the
//     // hub and therefore the robot.
//     // Gyro gets calibrated automatically in this mode.
//     imuParameters.mode = BNO055IMU.SensorMode.NDOF;
//     // Use degrees as angle unit.
//     imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//     // Express acceleration as m/s^2.
//     imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//     // Set acceleration integration algorithm to
//     // "naive" which will give velocity & position
//     imuParameters.accelerationIntegrationAlgorithm = null;
//     // Enable logging.
//     imuParameters.loggingEnabled = false;
//     // Initialize IMU.
//     imu.initialize(imuParameters);
//     // Initialize variables.
//     // Report the initialization to the Driver Station
//     telemetry.addData("IMU status", "IMU Initalized.  Calibration started....");
//     // Prompt user to press start buton.
//     telemetry.addData("IMU Quaternion example", "Press start to continue...");
//     telemetry.update();
//     waitForStart();
//     if (opModeIsActive()) {
//       // Put run blocks here.
//       imu.stopAccelerationIntegration();
//       while (opModeIsActive()) {
//         // Get quaternion from the IMU sensors
//         quat = imu.getQuaternionOrientation();
//         q0 = quat.w;
//         q1 = quat.x;
//         q2 = quat.y;
//         q3 = quat.z;
//         // convert quaternion to Euler angles.
//         // Since the IMU's parameters were
//         // set up for DEGREES, then the
//         // atan2 functions here will return
//         // values in DEGREES, not radians
//         // psi = yaw (or heading)
//         psi = Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) / Math.PI * 180;
//         // theta = pitch
//         theta = Math.asin(2 * (q0 * q2 - q3 * q1)) / Math.PI * 180;
//         // phi = roll
//         phi = Math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) / Math.PI * 180;
//         // post results
//         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//         foo = new ArrayList<Axis>(((OrientationSensor) imu).getAngularOrientationAxes());
//         gravity = imu.getGravity();
//         telemetry.addData("AngOrientationAxes", foo);
//         // Display orientation info.
//         telemetry.addData("rot about X", Double.parseDouble(JavaUtil.formatNumber(0 - angles.firstAngle, 1)));
//         telemetry.addData("roll (phi)", Double.parseDouble(JavaUtil.formatNumber(phi, 1)));
//         telemetry.addData("rot about Y", Double.parseDouble(JavaUtil.formatNumber(0 - angles.secondAngle, 1)));
//         telemetry.addData("pitch (theta)", Double.parseDouble(JavaUtil.formatNumber(theta, 1)));
//         telemetry.addData("rot about Z", Double.parseDouble(JavaUtil.formatNumber(0 + angles.thirdAngle, 1)));
//         telemetry.addData("yaw (psi)", Double.parseDouble(JavaUtil.formatNumber(psi, 1)));
//         telemetry.addData("q0", Double.parseDouble(JavaUtil.formatNumber(q0, 1)));
//         telemetry.addData("q1", Double.parseDouble(JavaUtil.formatNumber(q1, 1)));
//         telemetry.addData("q2", Double.parseDouble(JavaUtil.formatNumber(q2, 1)));
//         telemetry.addData("q3", Double.parseDouble(JavaUtil.formatNumber(q3, 1)));
//         // Display gravitational acceleration.
//         telemetry.addData("gravity (X)", gravity.xAccel);
//         telemetry.addData("gravity (Y)", gravity.yAccel);
//         telemetry.addData("gravity (Z)", gravity.zAccel);
//         telemetry.addData("calibration", imu.getCalibrationStatus());
//         telemetry.update();
//       }
//     }
//   }
// }
// 