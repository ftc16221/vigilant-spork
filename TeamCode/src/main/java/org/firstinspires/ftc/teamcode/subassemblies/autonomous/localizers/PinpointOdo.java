package org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;

public class PinpointOdo extends Localizer {

    public static double X_OFFSET = 0.0;
    public static double Y_OFFSET = 0.0;
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static GoBildaPinpointDriver.EncoderDirection X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;


    private final GoBildaPinpointDriver pinpoint;

    public PinpointOdo(LinearOpMode opMode, Pose startingPose) {
        super(opMode, "Pinpoint Odometry");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint_imu");

        pinpoint.setOffsets(X_OFFSET, Y_OFFSET, Global.DISTANCE_UNIT);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_DIRECTION, Y_DIRECTION);

        pinpoint.resetPosAndIMU();

        setPose(startingPose);
    }

    @Override public void update() {
        pose = new Pose(
                pinpoint.getPosX(Global.DISTANCE_UNIT),
                pinpoint.getPosY(Global.DISTANCE_UNIT),
                pinpoint.getHeading(Global.ANGLE_UNIT)
        );
        velocity = new Pose(
                pinpoint.getVelX(Global.DISTANCE_UNIT),
                pinpoint.getVelY(Global.DISTANCE_UNIT),
                pinpoint.getHeadingVelocity(Global.UNNORMALIZED_ANGLE_UNIT)
        );
    }

    @Override public void setPose(Pose newPose) {
        pose = newPose;
        pinpoint.setPosX(newPose.x, Global.DISTANCE_UNIT);
        pinpoint.setPosY(newPose.y, Global.DISTANCE_UNIT);
        pinpoint.setHeading(newPose.h, Global.ANGLE_UNIT);
    }

    public void recalibrate() {
        pinpoint.recalibrateIMU();
    }
}
