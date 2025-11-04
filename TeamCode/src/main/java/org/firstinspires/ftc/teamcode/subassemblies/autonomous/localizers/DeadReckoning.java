package org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

@Config
public class DeadReckoning extends Localizer {

    public static double CM_PER_REVOLUTION = 0;
    public static double ENCODER_RESOLUTION = 537.7;
    public static double TRACK_WIDTH = 0.0;
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    private final DcMotor leftFront, rightFront, leftRear, rightRear;
    private final IMU imu;
    private final Pose startingPose;

    private double prevTime = 0;
    private DcMotor[] motors;

    private ArrayList<Integer> previousPositions = new ArrayList<>();

    public DeadReckoning(OpMode opMode, MecDriveBase driveBase, Pose startingPose) {
        super(opMode, "Dead Reckoning");
        this.startingPose = startingPose;
        pose = startingPose;

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                HUB_LOGO_FACING_DIRECTION,
                                HUB_USB_FACING_DIRECTION
                        )
                )
        );

        leftFront = driveBase.getLeftFront();
        rightFront = driveBase.getRightFront();
        leftRear = driveBase.getLeftRear();
        rightRear = driveBase.getRightRear();
        motors = new DcMotor[]{leftFront, rightFront, leftRear, rightRear};
    }

    @Override
    public void update() {
        double time = opMode.time;

        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(Global.ANGLE_UNIT) + startingPose.h;

        Pose fieldCentricChange = getRelativeChange().toFieldCentric(pose.h);
        pose = pose.add(fieldCentricChange);

        double deltaTime = time - prevTime;
        velocity = fieldCentricChange.divideBy(deltaTime);

        prevTime = time;
    }

    /** @return x is forward, y is lateral, and h is rotational */
    private Pose getRelativeChange() {

        // put current motor positions into an arrayList
        ArrayList<Integer> currentPositions = new ArrayList<>();
        for (DcMotor motor: motors) {
            currentPositions.add(motor.getCurrentPosition());
        }
        // compare current positions to previous positions and put difference in an arrayList
        ArrayList<Integer> positionDelta = new ArrayList<>();
        for (int i = 0; i < currentPositions.size(); i++) {
            positionDelta.add(currentPositions.get(i) - previousPositions.get(i));
        }

        double deltaLF = toCM(positionDelta.get(0)); // leftFront
        double deltaRF = toCM(positionDelta.get(1)); // rightFront
        double deltaLR = toCM(positionDelta.get(2)); // leftRear
        double deltaRR = toCM(positionDelta.get(3)); // rightRear

        // math is according to gm0's forward kinematics: https://gm0.org/en/latest/docs/software/concepts/kinematics.html
        double deltaX = (deltaLF + deltaRF + deltaLR + deltaRR) / 4; // forward
        double deltaY = (deltaRF + deltaLR - deltaLF - deltaRR) / 4; // strafe
        double deltaH = (deltaRF + deltaRR - deltaLF - deltaLR) / (4 * TRACK_WIDTH); // rotation

        previousPositions = currentPositions;
        return new Pose(deltaX, deltaY, deltaH);
    }

    private double toCM(int encoderPos) {
        return encoderPos / ENCODER_RESOLUTION * CM_PER_REVOLUTION;
    }
}
