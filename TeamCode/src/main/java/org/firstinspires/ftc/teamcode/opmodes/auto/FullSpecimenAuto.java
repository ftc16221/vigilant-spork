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
@Autonomous(group = "full", preselectTeleOp = "SimpleTeleOp")
public class FullSpecimenAuto extends LinearOpMode {

    public static SparkFunOTOS.Pose2D startPose = new SparkFunOTOS.Pose2D(-63, -24, 0); // starting position
    public static SparkFunOTOS.Pose2D pickupPose = new SparkFunOTOS.Pose2D(-60, -50, -90); // TODO: find pickup position
    public static SparkFunOTOS.Pose2D score1Pose = new SparkFunOTOS.Pose2D(-40, -8, 90); // first scoring position
    public static SparkFunOTOS.Pose2D score2Pose = new SparkFunOTOS.Pose2D(-40, 0, 90); // second scoring position

    public static SparkFunOTOS.Pose2D pushHelperPose = new SparkFunOTOS.Pose2D(-40, -26, 90); // first push position
    public static SparkFunOTOS.Pose2D push1StartPose = new SparkFunOTOS.Pose2D(-6, -48, 90); // where the robot must start pushing the first sample
    public static SparkFunOTOS.Pose2D push2StartPose = new SparkFunOTOS.Pose2D(-6, -60, 90); // where the robot must start pushing the second sample
    public static SparkFunOTOS.Pose2D push1EndPose = new SparkFunOTOS.Pose2D(-60, -48, 90); // where the robot must end pushing the first sample
    public static SparkFunOTOS.Pose2D push2EndPose = new SparkFunOTOS.Pose2D(-55, -60, 90); // where the robot must end pushing the second sample

    public static double HIGH_RUNG_POS = 30;
    public static double PICKUP_POS = 6;

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
            pushSamples();
            pickUpSpecimen();
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
        follower.driveToPose(pushHelperPose, 5, false);
        follower.driveToPose(push1StartPose, 3, true);
        follower.driveToPose(push1EndPose, 3, true);
        follower.driveToPose(push1StartPose, 3, true);
        follower.driveToPose(push2StartPose, 3, true);
        follower.driveToPose(push2EndPose, 3, true);
    }
}
