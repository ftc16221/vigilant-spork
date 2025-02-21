package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subassemblies.AltClaw;
import org.firstinspires.ftc.teamcode.subassemblies.Follower;
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.AdvPose;
import org.firstinspires.ftc.teamcode.util.Global;

@Config
public class SemiAutoTeleOp extends LinearOpMode {

    // TODO: make this stuff global
    public static AdvPose basketScorePose = new AdvPose(-47, 65, 45, 2.5, true);
    public static AdvPose specimenScorePose = new AdvPose(-40, 0, 90, 2.5, true, true); // second scoring position


    @Override
    public void runOpMode() {
        MecDriveBase driveBase = new MecDriveBase(this);
        LinearSlide linearSlide = new LinearSlide(this);
        DcMotorEx linearSlideMotor = linearSlide.getLinearSlide();
        AltClaw claw = new AltClaw(this);
        Servo wristServo = claw.getRotateServo();
        Follower follower = new Follower(this, Global.lastPose);
        new Underglow(this);

        ElapsedTime loopTime = new ElapsedTime();

        boolean autoDrivebase = false;
        boolean autoScoreBasket = false;
        boolean autoScoreSpecimen = false;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                loopTime.reset();

                if (!gamepad1.atRest()) {
                    autoDrivebase = false;
                }

                if (!gamepad2.atRest()) {
                    autoScoreBasket = false;
                    autoScoreSpecimen = false;
                    linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                }

                driveBase.control(gamepad1);
                linearSlide.control(gamepad2);
                claw.control(gamepad2);

                // Semi-auto specimen scoring
                if (gamepad1.x || gamepad2.dpad_right) {
                    autoDrivebase = true;
                    autoScoreSpecimen = true;
                    Follower.USE_Y = false;
                    follower.setTargetPose(specimenScorePose);
                    linearSlide.moveSlide(linearSlide.HIGH_RUNG_POS, 1);
                } else {
                    Follower.USE_Y = true;
                }

                if (autoScoreSpecimen) {
                    if (!linearSlideMotor.isBusy() && !follower.isBusy()) {
                        wristServo.setPosition(0.3);
                        sleep(500);
                        linearSlide.moveSlide(linearSlide.HIGH_RUNG_POS - 3, 1);
                        sleep(500);
                        claw.open();
                        sleep(500);
                        wristServo.setPosition(0.8);
                        autoScoreSpecimen = false;
                    }
                }

                // Semi-auto basket scoring
                if (gamepad1.y || gamepad2.dpad_left) {
                    autoDrivebase = true;
                    autoScoreBasket = true;
                    follower.setTargetPose(basketScorePose);
                    linearSlide.moveSlide(linearSlide.HIGH_BASKET_POS, 1);
                }


                if (autoScoreBasket) {
                    if (linearSlideMotor.isBusy() && opModeIsActive()) {
                        telemetry.addData("linear slide pos", linearSlideMotor.getCurrentPosition());
                        telemetry.update();
                    } else {
                        wristServo.setPosition(0.4);
                        sleep(250);
                        claw.open();
                        sleep(250);
                        wristServo.setPosition(0.8);
                        sleep(400);
                        linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        autoScoreBasket = false;
                    }
                }

                if (autoDrivebase) {
                    follower.update();
                }
            }
        }
    }
}
