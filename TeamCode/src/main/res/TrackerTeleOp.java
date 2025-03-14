package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp
public class TrackerTeleOp extends LinearOpMode implements DashOpMode {

    public static SparkFunOTOS.Pose2D starting_pose = new SparkFunOTOS.Pose2D(0, 0, 0);

    Follower follower;
    MecDriveBase driveBase;
    LinearSlide linearSlide;
    AltClaw claw;
    SparkFunOTOS.Pose2D current_pose = starting_pose;
    Telemetry telemetryA;
    TelemetryPacket telemetryPacket;

    @Override
    public void runOpMode() {
        follower = new Follower(this, starting_pose);
        driveBase = new MecDriveBase(this);
        linearSlide = new LinearSlide(this);
        claw = new AltClaw(this);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryPacket = new TelemetryPacket();

        telemetryA.addLine("This opmode is used to find robot positions for autonomous. It is not intended" +
                "for driving the robot, but could be useful for debugging. Currently, the robot's position is purely" +
                "based off of the set starting position. If the starting position is not 0, 0, 0 (center of the field)" +
                "then you will need to change it via FTC Dashboard. I recommend 2 tiles left of the origin (-48, 0, 0)," +
                "because it is accessible and a good anchor point.");
        telemetryA.update();

        waitForStart();

        driveBase.control(gamepad1);
        linearSlide.control(gamepad2);
        claw.control(gamepad2);

        follower.telemetry();
        driveBase.telemetry();
        linearSlide.telemetry();
        claw.telemetry();
        telemetry.update();

        telemetryPacket.fieldOverlay()
                .drawImage("/TeamCode/src/main/res/drawable/cordelia", current_pose.x, current_pose.y, 18, 18) // draw cordelia at her position
                .drawImage("/TeamCode/src/main/res/drawable/arrow", current_pose.x, current_pose.y, 18, 18, current_pose.h, 0.5, 0, false); // draw an arrow on the robot's position, pointing toward the robot's heading, pivoting off the bottom middle of the image

        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }
}
