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

/**
 * Loosely based off of <a href="https://pedropathing.com/examples/auto.html">PedroPathing's Example Auto</a>
 */
@Config
@Autonomous(group = "part", preselectTeleOp = "Alt Claw TeleOp")
public class PartBasketAuto extends LinearOpMode {

    public static SparkFunOTOS.Pose2D startPose = new SparkFunOTOS.Pose2D(-61.8, 36, 0); // starting position
    public static SparkFunOTOS.Pose2D basketPose1 = new SparkFunOTOS.Pose2D(-47, 61, -135); // scoring position
    public static SparkFunOTOS.Pose2D basketPose2 = new SparkFunOTOS.Pose2D(-45, 59, -135); // scoring position
    public static SparkFunOTOS.Pose2D ascendPose1 = new SparkFunOTOS.Pose2D(-12, 43, 0); // first ascension position to avoid hitting submersible
    public static SparkFunOTOS.Pose2D ascendPose2 = new SparkFunOTOS.Pose2D(-12, 21.2, 0); // second and actual ascension position

    public static double SLIDE_HIGH_BASKET_POS = 38.4;
    public static int SLIDE_ASCEND_POS = 20;

    private Follower follower;
    private LinearSlide linearSlide;
    private DcMotor linearSlideMotor;
    private Servo pinionServo;
    private AltClaw claw;
    private Servo wristServo;

    @Override
    public void runOpMode() {
        follower = new Follower(this, startPose);

        linearSlide = new LinearSlide(this);
        linearSlideMotor = linearSlide.getLinearSlide();
        pinionServo = linearSlide.getPinion();

        claw = new AltClaw(this);
        wristServo = claw.getRotateServo();

        waitForStart();
        if (opModeIsActive()) {
            // score preloaded sample
            scoreSample();
            ascend();
            requestOpModeStop();
        }
    }

    private void scoreSample() {
        claw.close();
        wristServo.setPosition(0.8);
        linearSlide.moveSlide(SLIDE_HIGH_BASKET_POS, 1);
        follower.driveToPose(basketPose1, 2.5, true);
        while (linearSlideMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("linear slide pos", linearSlideMotor.getCurrentPosition());
            telemetry.update();
        }
        wristServo.setPosition(0.4);
        sleep(500);
        claw.open();
        sleep(500);
        wristServo.setPosition(0.8);
        sleep(500);
        follower.driveToPose(basketPose2, 2.5, false);
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
