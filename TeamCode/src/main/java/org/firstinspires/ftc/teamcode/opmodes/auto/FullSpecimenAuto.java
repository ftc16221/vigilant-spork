package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subassemblies.AltClaw;
import org.firstinspires.ftc.teamcode.subassemblies.Follower;
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.AdvPose;

import java.util.Arrays;
import java.util.List;

/**
 * Loosely based off of <a href="https://pedropathing.com/examples/auto.html">PedroPathing's Example Auto</a>
 */
@Disabled
@Config
@Autonomous(group = "full", preselectTeleOp = "Alt Claw TeleOp")
public class FullSpecimenAuto extends LinearOpMode {

    public static SparkFunOTOS.Pose2D startPose = new SparkFunOTOS.Pose2D(-61.8, -36, 0); // starting position

    public static AdvPose pickupPose = new AdvPose(-60, -50, -90, 2.5, true, true); // pickup position
    public static AdvPose score1Pose = new AdvPose(-40, -5, 90, 2.5, true, true); // first scoring position
    public static AdvPose score2Pose = new AdvPose(-40, 0, 90, 2.5, true, true); // second scoring position

    public static AdvPose pushHelperPose = new AdvPose(-36, -36, 90, 5, false); // first push position
    public static AdvPose push1StartPose = new AdvPose(-12, -50, 90, 3, true); // where the robot must start pushing the first sample
    public static AdvPose push2StartPose = new AdvPose(-12, -60, 90, 3, true); // where the robot must start pushing the second sample
    public static AdvPose push1EndPose = new AdvPose(-60, -50, 90, 3, true); // where the robot must end pushing the first sample
    public static AdvPose push2EndPose = new AdvPose(-50, -60, 90, 3, true); // where the robot must end pushing the second sample

    public static double HIGH_RUNG_POS = 30;
    public static double PICKUP_POS = 3;

    private final List<AdvPose> path = Arrays.asList(
            score1Pose, // score preload
            // push some samples to human player
            pushHelperPose,
            push1StartPose,
            push1EndPose,
            push1StartPose,
            push2StartPose,
            push2EndPose,
            // score those specimens
            pickupPose,
            score2Pose
    );

    private Follower follower;
    private LinearSlide linearSlide;
    private DcMotor linearSlideMotor;
    private Servo pinionServo;
    private AltClaw claw;
    private Servo wristServo;

    @Override
    public void runOpMode() {
        follower = new Follower(this, startPose);
        new Underglow(this);

        linearSlide = new LinearSlide(this);
        linearSlideMotor = linearSlide.getLinearSlide();
        pinionServo = linearSlide.getPinion();

        claw = new AltClaw(this);
        wristServo = claw.getRotateServo();
        follower.setPath(path);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                follower.update();
                switch(follower.getPathState()) {
                    case -1: // end of path
                        requestOpModeStop();
                        break;
                    case 0: // score1Pose
                        if (scoreSpecimen()) {
                            score1Pose.hold = false;
                        }
                        break;
                    case 1: // pushHelperPose
                        collapseSlides();
                        break;
                    case 2: case 3: case 4: case 5: case 6: // push1StartPose, push1EndPose, push1StartPose, push2StartPose, push2EndPose
                        // let drivebase do it's stuff
                        break;
                    case 7: // pickupPose
                        // pickup sample
                        if (pickUpSpecimen()) {
                            pickupPose.hold = false;
                        }
                        break;
                    case 8: // score2Pose
                        if (scoreSpecimen()) {
                            score2Pose.hold = false;
                        }
                        break;
                }
            }
            follower.stop();
        }
    }

    private boolean scoreSpecimen() {
        claw.close();
        linearSlide.moveSlide(HIGH_RUNG_POS, 1);
        if (!linearSlideMotor.isBusy() && follower.isNotBusy()) {
            wristServo.setPosition(0.3);
            sleep(500);
            linearSlide.moveSlide(HIGH_RUNG_POS - 3, 1);
            sleep(500);
            claw.open();
            sleep(500);
            wristServo.setPosition(0.8);
            return true;
        } else {
            return false;
        }
    }

    private boolean pickUpSpecimen() {
        // do something
        return true;
    }

    private void collapseSlides() {
        linearSlide.moveSlide(0, 1);
        wristServo.setPosition(0.7);
    }
}
