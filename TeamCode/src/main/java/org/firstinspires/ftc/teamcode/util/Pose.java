package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Pose {
    public double x, y , h;

    public Pose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public Pose(SparkFunOTOS.Pose2D sparkFunPose) {
        this.x = sparkFunPose.x;
        this.y = sparkFunPose.y;
        this.h = sparkFunPose.h;
    }

    public Pose(Pose3D pose3D) {
        Position position = pose3D.getPosition();
        this.x = position.x;
        this.y = position.y;
        this.h = pose3D.getOrientation().getYaw(Global.ANGLE_UNIT);
    }

    public SparkFunOTOS.Pose2D toSparkFunPose() {
        return new SparkFunOTOS.Pose2D(x, y, h);
    }

    public void draw(String color) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        double hInRadians = Math.toRadians(h);

        packet.fieldOverlay()
                .setStroke(color)
                .setRotation(Math.toRadians(hInRadians))
                .strokeCircle(x, y, 9)
                .strokeLine(x, y, x + (9 * Math.cos(hInRadians)), y + (9 * Math.sin(hInRadians)));

        dashboard.sendTelemetryPacket(packet);
    }

}
