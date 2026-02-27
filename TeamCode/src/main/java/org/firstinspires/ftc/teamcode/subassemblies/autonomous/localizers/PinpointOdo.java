package org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * <a href="https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf">User Guide</a>
 */
//@Config
public class PinpointOdo extends Localizer {

    // Both of these offsets are in MM. The tracking point of the robot is the center of the wheelbase
    public static double X_OFFSET = 72; // 72mm
    public static double Y_OFFSET = 60; // 60mm
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static GoBildaPinpointDriver.EncoderDirection X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public final GoBildaPinpointDriver pinpoint;

    public PinpointOdo(OpMode opMode, Pose startingPose) {
        super(opMode, "Pinpoint Odometry", false, 0.98);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint_imu");

        pinpoint.setOffsets(X_OFFSET, Y_OFFSET, DistanceUnit.MM);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_DIRECTION, Y_DIRECTION);

        pinpoint.recalibrateIMU();

        setPose(startingPose);
    }

    @Override
    public void setPose(Pose newPose) {
        if (newPose != null) {
            pose = newPose;
            pinpoint.setPosition(pose.toPose2D());
            Watchdog.i("pinpoint pose has been set to " + newPose.toPose2D().toString());
        }
    }

    @Override
    public void update() {
        pinpoint.update();
        pose = new Pose(pinpoint.getPosition());
    }

    /*@Override*/
    public List<String> findIssues() {
        List<String> issues = new ArrayList<>();
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (status != GoBildaPinpointDriver.DeviceStatus.READY && status != GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
            Watchdog.w("Pinpoint status is " + status.name() + ", odometry functionality is likely very limited");
        }
        return issues;
    }

    public void recalibrate() {
        pinpoint.recalibrateIMU();
    }
}
