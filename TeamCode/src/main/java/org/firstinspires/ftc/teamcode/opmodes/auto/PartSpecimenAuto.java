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
public class PartSpecimenAuto extends LinearOpMode {

    public static SparkFunOTOS.Pose2D startPose = new SparkFunOTOS.Pose2D(-61.8, -36, 0); // starting position
    public static SparkFunOTOS.Pose2D score1Pose = new SparkFunOTOS.Pose2D(-40, -5, 90); // first scoring position

    public static double HIGH_RUNG_POS = 30;

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
            scoreSpecimen(score1Pose);
            requestOpModeStop();
        }
    }

    private void scoreSpecimen(SparkFunOTOS.Pose2D pose) {
        claw.close();
        pinionServo.setPosition(0.2);
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
