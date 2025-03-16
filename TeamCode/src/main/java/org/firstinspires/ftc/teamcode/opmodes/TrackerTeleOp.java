package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subassemblies.AltClaw;
import org.firstinspires.ftc.teamcode.subassemblies.Follower;
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.util.DashOpMode;

@Config
@TeleOp(group = "tests")
public class TrackerTeleOp extends LinearOpMode implements DashOpMode {

    public static SparkFunOTOS.Pose2D startingPose = new SparkFunOTOS.Pose2D(0, 0, 0);

    Follower follower;
    MecDriveBase driveBase;
    LinearSlide linearSlide;
    AltClaw claw;
    SparkFunOTOS.Pose2D currentPose = startingPose;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new Follower(this, startingPose);
        driveBase = new MecDriveBase(this);
        linearSlide = new LinearSlide(this);
        claw = new AltClaw(this);

        telemetry.addLine("This opmode is used to find robot positions for autonomous. It is not intended " +
                "for driving the robot, but may be useful for debugging. Currently, the robot's position is purely " +
                "based off of the set starting position. If the starting position is not 0, 0, 0 (center of the field) " +
                "then you will need to change it via FTC Dashboard. I recommend 2 tiles left of the origin (-48, 0, 0), " +
                "because it is accessible and a good anchor point.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                driveBase.control(gamepad1);
                linearSlide.control(gamepad2);
                claw.control(gamepad2);

                currentPose = follower.getCurrentPose();

                follower.telemetry();
                driveBase.telemetry();
                linearSlide.telemetry();
                claw.telemetry();
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket(true);
                packet.fieldOverlay()
                        .setStroke("#12C600")
                        .setRotation(Math.toRadians(currentPose.h))
                        .setTranslation(currentPose.y, -currentPose.x) // x and y are swapped because FTC dash's coordinate system wants to be different
                        .strokeCircle(0, 0, 9) // draw circle for robot position
                        .strokeLine(0, 0, 9, 0);

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }
    }
}
