package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.LocalizationManager;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.PinpointOdo;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Path;
import org.firstinspires.ftc.teamcode.util.Pose;

@Autonomous(name = "BioBuzz Four Pollen Startbot Auto", group = "StarterBot")
@Config
public class StarterBotFourPollenAuto extends OpMode {

    public static Pose STARTING_POSE = new Pose(0, 0, 0);
    public static Pose POSE_1 = new Pose(0, -95, 0);
    public static Pose POSE_2 = new Pose(-180, -95, 0);
    public static Pose POSE_3 = new Pose(-180, -105, 0);

    private Navigator navigator;
    private Watchdog watchdog;
    private PinpointOdo pinpointOdo;
    private DcMotorEx launcher = null;
    private CRServo windmillServo = null;

    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;

    private DcMotor intake = null;
    public static int LAUNCHER_TARGET_VELOCITY = 1275; // Ticks / second
    public static int LAUNCHER_MIN_VELOCITY = 1225;

    private State state = State.NOT_STARTED;

    boolean launching = false;

    double intakePower;
    ElapsedTime timer = new ElapsedTime();
    private Path path1;

    @Override
    public void init() {
        pinpointOdo = new PinpointOdo(this, STARTING_POSE);
        LocalizationManager localizationManager = new LocalizationManager(this, pinpointOdo);
        navigator = new Navigator(this, localizationManager);
        watchdog = new Watchdog(this);
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        windmillServo.setPower(0);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        path1 = new Path(POSE_1, POSE_2, POSE_3);
        navigator.setTargetPose(STARTING_POSE);


    }
    @Override
    public void start() {
        navigator.start();
        navigator.enableMovement();
        timer.reset();
    }

    @Override
    public void loop() {



        switch (state) {
            case NOT_STARTED:
                state = State.LAUNCHING;
                break;
            case LAUNCHING:
                if (!launching) {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    launching = true;
                }
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    windmillServo.setPower(1);

                    intakePower = 0.5;
                    intake.setPower(intakePower);
                    leftIntakeServo.setPower(intakePower);
                    rightIntakeServo.setPower(intakePower);
                }
                if (timer.seconds() > 5) {
                    launcher.setVelocity(0);
                    windmillServo.setPower(0);

                    intakePower = 0;
                    intake.setPower(0);
                    leftIntakeServo.setPower(0);
                    rightIntakeServo.setPower(0);
                    state = State.MOVING;
                }

                //telemetry.addData("Flywheel Speed (TPS)", launcher.getVelocity());
                break;
            case MOVING:
                if (!path1.isComplete()) {
                    path1.execute(navigator);
                }
                else {
                    state = State.DONE;
                }
                break;
            case DONE:
                requestOpModeStop();

        }

        navigator.update();
        watchdog.update();
        navigator.runTelemetry();
        telemetry.addData("state", state);
        telemetry.addData("navigator isAtTarget", navigator.isAtTarget());
        telemetry.addData("path is complete", path1.isComplete());
        telemetry.addData("current pos", navigator.getCurrentPose().toString());
        if (navigator.getTargetPose() != null) {
            telemetry.addData("target pos", navigator.getTargetPose().toString());
        } else {
            telemetry.addData("target pos", null);
        }

    }

    private enum State {
        NOT_STARTED, LAUNCHING, MOVING, DONE
    }



}
