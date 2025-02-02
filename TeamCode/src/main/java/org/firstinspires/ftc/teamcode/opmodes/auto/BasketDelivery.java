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

@Config
@Autonomous(name = "Basket Delivery", group = "OTOS", preselectTeleOp = "Alt Claw TeleOp")
public class BasketDelivery extends LinearOpMode {

    public static SparkFunOTOS.Pose2D STARTING_POSITION = new SparkFunOTOS.Pose2D(-61.8, 36, 0);
    public static double HIGH_BASKET_POS = 38.4;
    public static int ASCEND_POS = 40;

    private Follower follower;
    private LinearSlide linearSlide;
    private DcMotor linearSlideMotor;
    private Servo pinionServo;
    private AltClaw claw;
    private Servo wristServo;

    @Override
    public void runOpMode() {
        follower = new Follower(this, STARTING_POSITION);

        linearSlide = new LinearSlide(this);
        linearSlideMotor = linearSlide.getLinearSlide();
        pinionServo = linearSlide.getPinion();

        claw = new AltClaw(this);
        wristServo = claw.getRotateServo();

        waitForStart();
        if (opModeIsActive()) {
            follower.driveToPos(
                    -47,
                    61,
                    -135,
                    2.5,
                    true
            );
            scoreSpecimen();
            follower.driveToPos(
                    -48,
                    36,
                    -135,
                    10,
                    false
            );
            follower.driveToPos(
                    -12,
                    43,
                    0,
                    10,
                    false
            );
            follower.driveToPos(
                    -12,
                    21.2,
                    0,
                    2,
                    true
            );
        }
    }

    private void scoreSpecimen() {
        wristServo.setPosition(0.8);
        linearSlide.moveSlide(HIGH_BASKET_POS, 1);
        while (linearSlideMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("linear slide pos", linearSlideMotor.getCurrentPosition());
            telemetry.update();
        }
        linearSlideMotor.setPower(1);
        wristServo.setPosition(0.4);
        sleep(500);
        claw.open();
        sleep(500);
        wristServo.setPosition(0.8);
        sleep(500);
        linearSlide.moveSlide(ASCEND_POS, 1);
    }
}
