package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subassemblies.AltClaw;
import org.firstinspires.ftc.teamcode.subassemblies.Follower;
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.AdvPose;
import org.firstinspires.ftc.teamcode.util.Global;

@Disabled
@Config
@TeleOp(group = "!main")
public class SemiAutoTeleOp extends LinearOpMode {

    // TODO: make this stuff global
    public static AdvPose basketScorePose = new AdvPose(-47, 65, 45, 2.5, true);
    public static AdvPose specimenScorePose = new AdvPose(-40, 0, 90, 2.5, true, true); // second scoring position

    public static double HIGH_RUNG_POS = 19;
    public static double POS_DIFFERENCE = 6;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        MecDriveBase driveBase = new MecDriveBase(this);
        LinearSlide linearSlide = new LinearSlide(this);
        DcMotorEx linearSlideMotor = linearSlide.getLinearSlide();
        AltClaw claw = new AltClaw(this);
        Servo wristServo = claw.getRotateServo();
        new Underglow(this);

        ElapsedTime loopTime = new ElapsedTime();

        boolean autoScoreBasket = false;
        boolean autoScoreSpecimen = false;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                loopTime.reset();

                if (!gamepad2.atRest()) {
                    autoScoreSpecimen = false;
                    linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                }

                driveBase.control(gamepad1);
                if (!autoScoreBasket && !autoScoreSpecimen) {
                    linearSlide.control(gamepad2);
                    claw.control(gamepad2);
                }

                // Semi-auto specimen scoring
                if (gamepad1.x || gamepad2.dpad_right) {
                    autoScoreSpecimen = true;
                    linearSlide.moveSlide(linearSlide.HIGH_RUNG_POS, 1);
                }

                if (autoScoreSpecimen) {
                    if (!linearSlideMotor.isBusy()) {
                        wristServo.setPosition(0.3);
                        sleep(750);
                        linearSlide.moveSlide(HIGH_RUNG_POS - POS_DIFFERENCE, 1);
                        sleep(500);
                        claw.open();
                        sleep(400);
                        wristServo.setPosition(0.8);
                    }
                }

                // Semi-auto basket scoring
                if (gamepad1.y || gamepad2.dpad_left) {
                    autoScoreBasket = true;
                    claw.close();
                    linearSlide.moveSlide(HIGH_RUNG_POS, 1);
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
            }
        }
    }
}
