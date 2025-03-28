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
    public static SparkFunOTOS.Pose2D basketPose2 = new SparkFunOTOS.Pose2D(-43, 63.5, 45); // scoring position
    public static SparkFunOTOS.Pose2D pickup1Pose = new SparkFunOTOS.Pose2D(-34, 42, -90); // first sample pickup
    public static SparkFunOTOS.Pose2D pickup2Pose = new SparkFunOTOS.Pose2D(-34, 53, -90); // second sample pickup
    public static SparkFunOTOS.Pose2D ascendPose1 = new SparkFunOTOS.Pose2D(-12, 43, 180); // first ascension position to avoid hitting submersible
    public static SparkFunOTOS.Pose2D ascendPose2 = new SparkFunOTOS.Pose2D(-12, 21.2, 180); // second and actual ascension position

    public static double SLIDE_HIGH_BASKET_POS = 38.4;
    public static double SLIDE_PICKUP_POS = 1;
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
