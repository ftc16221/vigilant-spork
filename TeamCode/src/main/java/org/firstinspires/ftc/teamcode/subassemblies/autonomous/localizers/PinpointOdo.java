package org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers;

import static org.firstinspires.ftc.teamcode.util.MathKt.normalize;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;

/** <a href="https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf">User Guide</a> */
@Config
public class PinpointOdo extends Localizer {

    // Both of these offsets are in MM. The tracking point of the robot isn't the center: it is the very front of the chassis.
    public static double X_OFFSET = 35.0;
    public static double Y_OFFSET = -55.0;
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static GoBildaPinpointDriver.EncoderDirection X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;


    public final GoBildaPinpointDriver pinpoint;

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
        pinpoint.update();

        pose = new Pose(
                pinpoint.getPosX(Global.DISTANCE_UNIT),
                pinpoint.getPosY(Global.DISTANCE_UNIT),
                normalize(pinpoint.getHeading(Global.UNNORMALIZED_ANGLE_UNIT)) // i'm not sure why, but when a normalized angle unit is used instead the value outputs as degrees but normalizes to the same range as radians
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
