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
    public static SparkFunOTOS.Pose2D pickupPose = new SparkFunOTOS.Pose2D(-54, -40, 90); // pickup position
    public static SparkFunOTOS.Pose2D scoreHelperPose = new SparkFunOTOS.Pose2D(-48, -24, -90); // helper scoring pose
    public static SparkFunOTOS.Pose2D score1Pose = new SparkFunOTOS.Pose2D(-39, -8, -90); // first scoring position
    public static SparkFunOTOS.Pose2D score2Pose = new SparkFunOTOS.Pose2D(-39, -6, -90); // second scoring position
    public static SparkFunOTOS.Pose2D score3Pose = new SparkFunOTOS.Pose2D(-39, -4, -90); // third scoring position
    public static SparkFunOTOS.Pose2D score4Pose = new SparkFunOTOS.Pose2D(-39, -2, -90); // fourth scoring position
    public static SparkFunOTOS.Pose2D parkPose = new SparkFunOTOS.Pose2D(-54, -45, -90); // pickup position

    public static SparkFunOTOS.Pose2D pushHelperPose1 = new SparkFunOTOS.Pose2D(-40, -40, 0); // first push position
    public static SparkFunOTOS.Pose2D pushHelperPose2 = new SparkFunOTOS.Pose2D(-6, -40, 90); // first push position
    public static SparkFunOTOS.Pose2D push1StartPose = new SparkFunOTOS.Pose2D(-6, -48, 90); // where the robot must start pushing the first sample
    public static SparkFunOTOS.Pose2D push2StartPose = new SparkFunOTOS.Pose2D(-6, -60, 90); // where the robot must start pushing the second sample
    public static SparkFunOTOS.Pose2D push1EndPose = new SparkFunOTOS.Pose2D(-60, -48, 90); // where the robot must end pushing the first sample
    public static SparkFunOTOS.Pose2D push2EndPose = new SparkFunOTOS.Pose2D(-60, -60, 90); // where the robot must end pushing the second sample

    public static double HIGH_RUNG_POS = 19;
    public static double POS_DIFFERENCE = 6;
    public static double PICKUP_POS = 5;

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
            scoreSpecimen(score1Pose);
            pushSamples();
            pickUpSpecimen();
            follower.driveToPose(scoreHelperPose, 10, false);
            scoreSpecimen(score2Pose);

            pickUpSpecimen();
            follower.driveToPose(scoreHelperPose, 10, false);
            scoreSpecimen(score3Pose);

            pickUpSpecimen();
            follower.driveToPose(scoreHelperPose, 10, false);
            scoreSpecimen(score4Pose);

            park();
        }
        follower.stop();
    }

    private void park() {
        follower.driveToPose(parkPose, 5, true);
        requestOpModeStop();
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
        sleep(750);
        linearSlide.moveSlide(HIGH_RUNG_POS - POS_DIFFERENCE, 1);
        sleep(500);
        claw.open();
        sleep(400);
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
        sleep(250);
        linearSlide.moveSlide(PICKUP_POS + 5, 1);
        sleep(250);
        wristServo.setPosition(0.6);
    }

    private void pushSamples() {
        linearSlide.moveSlide(0, 1);
        follower.driveToPose(pushHelperPose1, 5, false);
        follower.driveToPose(pushHelperPose2, 5, false);
        follower.driveToPose(push1StartPose, 8, false);
        follower.driveToPose(push1EndPose, 8, false);
        follower.driveToPose(push1StartPose, 8, false);
        follower.driveToPose(push2StartPose, 8, false);
        follower.driveToPose(push2EndPose, 8, false);
    }
}
