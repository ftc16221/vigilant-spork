I didn't want to delete past examples of autonomous so we can easily whip something up so I'm keeping them here for easy access. I'm sure there is a better way of doing this but if it works it works

### FullBasketAuto:

package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subassemblies.AltClaw;
import org.firstinspires.ftc.teamcode.subassemblies.Follower;
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

/**
* Loosely based off of <a href="https://pedropathing.com/examples/auto.html">PedroPathing's Example Auto</a>
  */
  @Config
  @Autonomous(group = "full", preselectTeleOp = Global.PRESELECT_TELEOP)
  public class FullBasketAuto extends LinearOpMode {

  public static SparkFunOTOS.Pose2D startPose = new SparkFunOTOS.Pose2D(-61.8, 36, 180); // starting position
  public static SparkFunOTOS.Pose2D basketPose1 = new SparkFunOTOS.Pose2D(-43, 60, -135); // scoring position helper
  public static SparkFunOTOS.Pose2D basketPose2 = new SparkFunOTOS.Pose2D(-47, 65, 45); // scoring position
  public static SparkFunOTOS.Pose2D pickup1Pose = new SparkFunOTOS.Pose2D(-31, 42, -90); // first sample pickup
  public static SparkFunOTOS.Pose2D pickup2Pose = new SparkFunOTOS.Pose2D(-31, 52, -90); // second sample pickup
  public static SparkFunOTOS.Pose2D ascendPose1 = new SparkFunOTOS.Pose2D(-12, 43, 180); // first ascension position to avoid hitting submersible
  public static SparkFunOTOS.Pose2D ascendPose2 = new SparkFunOTOS.Pose2D(-12, 21.2, 180); // second and actual ascension position

  public static double SLIDE_HIGH_BASKET_POS = 38.4;
  public static double SLIDE_PICKUP_POS = 3;
  public static int SLIDE_ASCEND_POS = 20;

  private Follower follower;
  private LinearSlide linearSlide;
  private DcMotor linearSlideMotor;
  private AltClaw claw;
  private Servo wristServo;

  @Override
  public void runOpMode() {
  claw = new AltClaw(this);
  wristServo = claw.getRotateServo();
  claw.close();

       follower = new Follower(this, startPose);
       new Underglow(this);

       linearSlide = new LinearSlide(this);
       linearSlideMotor = linearSlide.getLinearSlide();

       linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       sleep(10);
       linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       waitForStart();
       if (opModeIsActive()) {
           // score preloaded sample
           scoreSample();
           // pickup sample
           pickUpSample(pickup1Pose);
           // try to score again
           scoreSampleHelper();
           // pickup sample again
           pickUpSample(pickup2Pose);
           // try to score again
           scoreSampleHelper();
           // go in for ascension
           ascend();
           requestOpModeStop();
       }
       follower.stop();
  }

  private void scoreSample() {
  claw.close();
  wristServo.setPosition(0.7);
  linearSlide.moveSlide(SLIDE_HIGH_BASKET_POS, 1);
  follower.driveToPose(basketPose2, 5, true);
  while (linearSlideMotor.isBusy() && opModeIsActive()) {
  telemetry.addData("linear slide pos", linearSlideMotor.getCurrentPosition());
  telemetry.update();
  }
  wristServo.setPosition(0.4);
  sleep(250);
  claw.open();
  sleep(250);
  wristServo.setPosition(0.8);
  sleep(400);
  }

  private void scoreSampleHelper() {
  claw.close();
  wristServo.setPosition(0.7);
  linearSlide.moveSlide(SLIDE_HIGH_BASKET_POS, 1);
  follower.driveToPose(basketPose1, 5, false);
  scoreSample();
  }

  private void pickUpSample(SparkFunOTOS.Pose2D pose) {
  linearSlide.moveSlide(SLIDE_PICKUP_POS, 1);
  claw.open();
  wristServo.setPosition(0);
  follower.driveToPose(pose, 3, true);
  while (linearSlideMotor.isBusy() && opModeIsActive()) {
  telemetry.addData("linear slide pos", linearSlideMotor.getCurrentPosition());
  telemetry.update();
  }
  claw.close();
  sleep(500);
  }

  private void ascend() {
  linearSlide.moveSlide(SLIDE_ASCEND_POS, 1);
  follower.driveToPose(ascendPose1, 5, false);
  follower.driveToPose(ascendPose2, 3, true);
  while (linearSlideMotor.isBusy() && opModeIsActive()) {
  telemetry.addData("linear slide pos", linearSlideMotor.getCurrentPosition());
  telemetry.update();
  }
  }
  }


### FullSpecimenAuto:

package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subassemblies.AltClaw;
import org.firstinspires.ftc.teamcode.subassemblies.Follower;
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

/**
* Loosely based off of <a href="https://pedropathing.com/examples/auto.html">PedroPathing's Example Auto</a>
  */
  @Config
  @Autonomous(group = "full", preselectTeleOp = Global.PRESELECT_TELEOP)
  public class FullSpecimenAuto extends LinearOpMode {

  public static SparkFunOTOS.Pose2D startPose = new SparkFunOTOS.Pose2D(-63, -24, 180); // starting position
  public static SparkFunOTOS.Pose2D pickupPose = new SparkFunOTOS.Pose2D(-56, -40, 90); // TODO: find pickup position
  public static SparkFunOTOS.Pose2D scoreHelperPose = new SparkFunOTOS.Pose2D(-48, -24, -90); // helper scoring pose
  public static SparkFunOTOS.Pose2D score1Pose = new SparkFunOTOS.Pose2D(-40, -8, -90); // first scoring position
  public static SparkFunOTOS.Pose2D score2Pose = new SparkFunOTOS.Pose2D(-40, 0, -90); // second scoring position

  public static SparkFunOTOS.Pose2D pushHelperPose1 = new SparkFunOTOS.Pose2D(-40, -40, 0); // first push position
  public static SparkFunOTOS.Pose2D pushHelperPose2 = new SparkFunOTOS.Pose2D(-6, -40, 90); // first push position
  public static SparkFunOTOS.Pose2D push1StartPose = new SparkFunOTOS.Pose2D(-6, -48, 90); // where the robot must start pushing the first sample
  public static SparkFunOTOS.Pose2D push2StartPose = new SparkFunOTOS.Pose2D(-6, -60, 90); // where the robot must start pushing the second sample
  public static SparkFunOTOS.Pose2D push1EndPose = new SparkFunOTOS.Pose2D(-55, -48, 90); // where the robot must end pushing the first sample
  public static SparkFunOTOS.Pose2D push2EndPose = new SparkFunOTOS.Pose2D(-55, -60, 90); // where the robot must end pushing the second sample

  public static double HIGH_RUNG_POS = 30;
  public static double PICKUP_POS = 5;

  private Follower follower;
  private LinearSlide linearSlide;
  private DcMotor linearSlideMotor;
  private AltClaw claw;
  private Servo wristServo;

  @Override
  public void runOpMode() {
  claw = new AltClaw(this);
  wristServo = claw.getRotateServo();
  claw.close();

       follower = new Follower(this, startPose);
       new Underglow(this);

       linearSlide = new LinearSlide(this);
       linearSlideMotor = linearSlide.getLinearSlide();

       waitForStart();
       if (opModeIsActive()) {
           scoreSpecimen(score1Pose);
           pushSamples();
           pickUpSpecimen();
           follower.driveToPose(scoreHelperPose, 10, false);
           scoreSpecimen(score2Pose);
       }
       follower.stop();
  }

  private void scoreSpecimen(SparkFunOTOS.Pose2D pose) {
  claw.close();
  linearSlide.moveSlide(HIGH_RUNG_POS, 1);
  follower.driveToPose(pose, 2.5, true);
  while(linearSlideMotor.isBusy()) {
  telemetry.addData("Linear Slide Position", linearSlideMotor.getCurrentPosition());
  telemetry.update();
  }
  wristServo.setPosition(0.3);
  sleep(500);
  linearSlide.moveSlide(HIGH_RUNG_POS - 3, 1);
  sleep(500);
  claw.open();
  sleep(500);
  wristServo.setPosition(0.8);
  }

  private void pickUpSpecimen() {
  claw.open();
  linearSlide.moveSlide(PICKUP_POS, 1);
  follower.driveToPose(pickupPose, 2.5, true);
  while(linearSlideMotor.isBusy()) {
  telemetry.addData("Linear Slide Position", linearSlideMotor.getCurrentPosition());
  telemetry.update();
  }
  wristServo.setPosition(0.3);
  sleep(500);
  claw.close();
  }

  private void pushSamples() {
  follower.driveToPose(pushHelperPose1, 5, false);
  follower.driveToPose(pushHelperPose2, 5, false);
  follower.driveToPose(push1StartPose, 8, true);
  follower.driveToPose(push1EndPose, 8, true);
  follower.driveToPose(push1StartPose, 8, true);
  follower.driveToPose(push2StartPose, 8, true);
  follower.driveToPose(push2EndPose, 8, true);
  }
  }


### PartBasketAuto

package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subassemblies.AltClaw;
import org.firstinspires.ftc.teamcode.subassemblies.Follower;
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

/**
* Loosely based off of <a href="https://pedropathing.com/examples/auto.html">PedroPathing's Example Auto</a>
  */
  @Config
  @Autonomous(group = "part", preselectTeleOp = Global.PRESELECT_TELEOP)
  public class PartBasketAuto extends LinearOpMode {

  public static SparkFunOTOS.Pose2D startPose = new SparkFunOTOS.Pose2D(-61.8, 36, 180); // starting position
  public static SparkFunOTOS.Pose2D basketPose1 = new SparkFunOTOS.Pose2D(-43, 60, -135); // scoring position helper
  public static SparkFunOTOS.Pose2D basketPose2 = new SparkFunOTOS.Pose2D(-47, 65, 45); // scoring position
  public static SparkFunOTOS.Pose2D ascendPose1 = new SparkFunOTOS.Pose2D(-12, 43, 180); // first ascension position to avoid hitting submersible
  public static SparkFunOTOS.Pose2D ascendPose2 = new SparkFunOTOS.Pose2D(-12, 21.2, 180); // second and actual ascension position

  public static double SLIDE_HIGH_BASKET_POS = 38.4;
  public static int SLIDE_ASCEND_POS = 20;

  private Follower follower;
  private LinearSlide linearSlide;
  private DcMotor linearSlideMotor;
  private AltClaw claw;
  private Servo wristServo;

  @Override
  public void runOpMode() {
  follower = new Follower(this, startPose);
  new Underglow(this);

       linearSlide = new LinearSlide(this);
       linearSlideMotor = linearSlide.getLinearSlide();

       claw = new AltClaw(this);
       wristServo = claw.getRotateServo();

       waitForStart();
       if (opModeIsActive()) {
           // score preloaded sample
           scoreSample();
           ascend();
           follower.stop();
           requestOpModeStop();
       }
  }

  private void scoreSample() {
  claw.close();
  wristServo.setPosition(0.7);
  linearSlide.moveSlide(SLIDE_HIGH_BASKET_POS, 1);
  follower.driveToPose(basketPose2, 5, true);
  while (linearSlideMotor.isBusy() && opModeIsActive()) {
  telemetry.addData("linear slide pos", linearSlideMotor.getCurrentPosition());
  telemetry.update();
  }
  wristServo.setPosition(0.4);
  sleep(250);
  claw.open();
  sleep(250);
  wristServo.setPosition(0.8);
  sleep(400);
  }

  private void ascend() {
  linearSlide.moveSlide(SLIDE_ASCEND_POS, 1);
  follower.driveToPose(ascendPose1, 2.5, true);
  follower.driveToPose(ascendPose2, 2.5, true);
  while (linearSlideMotor.isBusy() && opModeIsActive()) {
  telemetry.addData("linear slide pos", linearSlideMotor.getCurrentPosition());
  telemetry.update();
  }
  }
  }


### PartSpeciment Auto

package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subassemblies.AltClaw;
import org.firstinspires.ftc.teamcode.subassemblies.Follower;
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

/**
* Loosely based off of <a href="https://pedropathing.com/examples/auto.html">PedroPathing's Example Auto</a>
  */
  @Config
  @Autonomous(group = "part", preselectTeleOp = Global.PRESELECT_TELEOP)
  public class PartSpecimenAuto extends LinearOpMode {

  public static SparkFunOTOS.Pose2D startPose = new SparkFunOTOS.Pose2D(-61.8, -36, 0); // starting position
  public static SparkFunOTOS.Pose2D score1Pose = new SparkFunOTOS.Pose2D(-40, -5, 90); // first scoring position

  public static double HIGH_RUNG_POS = 30;

  private Follower follower;
  private LinearSlide linearSlide;
  private DcMotor linearSlideMotor;
  private AltClaw claw;
  private Servo wristServo;

  @Override
  public void runOpMode() {
  follower = new Follower(this, startPose);
  new Underglow(this);

       linearSlide = new LinearSlide(this);
       linearSlideMotor = linearSlide.getLinearSlide();

       claw = new AltClaw(this);
       wristServo = claw.getRotateServo();

       waitForStart();
       if (opModeIsActive()) {
           scoreSpecimen(score1Pose);
           follower.stop();
           requestOpModeStop();
       }
  }

  private void scoreSpecimen(SparkFunOTOS.Pose2D pose) {
  claw.close();
  linearSlide.moveSlide(HIGH_RUNG_POS, 1);
  follower.driveToPose(pose, 2.5, true);
  while(linearSlideMotor.isBusy()) {
  telemetry.addData("Linear Slide Position", linearSlideMotor.getCurrentPosition());
  telemetry.update();
  }
  wristServo.setPosition(0.3);
  sleep(500);
  linearSlide.moveSlide(HIGH_RUNG_POS - 3, 1);
  sleep(500);
  claw.open();
  sleep(500);
  wristServo.setPosition(0.8);
  }
  }
