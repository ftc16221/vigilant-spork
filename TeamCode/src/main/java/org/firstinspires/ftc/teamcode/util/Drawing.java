package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Path;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;

@Config
public class Drawing {

    public static String CURRENT_POSE_COLOR = "yellow";
    public static String TARGET_POSE_COLOR = "green";
    public static String PATH_COLOR = "blue";
    public static int POSE_RADIUS = 30;
    public static int STROKE_WIDTH = 5;

    public final double INCH_PER_CM = 0.393701;

    private final FtcDashboard dashboard;
    private final Navigator navigator;
    private Canvas canvas;

    Path path;

    private boolean enableCurrentPose = true;
    private boolean enableTargetPose = true;
    private boolean enablePath = false;

    public Drawing(Navigator navigator) {
        this.navigator = navigator;
        dashboard = FtcDashboard.getInstance();
    }

    public void update() {
        TelemetryPacket packet = new TelemetryPacket();
        canvas = packet.fieldOverlay();
        canvas.setScale(INCH_PER_CM, INCH_PER_CM);
        canvas.setStrokeWidth(STROKE_WIDTH);

        if (enablePath && path != null)
            drawPath(path, PATH_COLOR);
        if (enableTargetPose && navigator.getTargetPose() != null)
            drawPose(navigator.getTargetPose(), TARGET_POSE_COLOR);
        if (enableCurrentPose && navigator.getCurrentPose() != null)
            drawPose(navigator.getCurrentPose(), CURRENT_POSE_COLOR);

        dashboard.sendTelemetryPacket(packet);
    }

    public void drawPath(Path path, String color) {
        Pose[] poses = path.get();
        for (int i = 0; i < poses.length; i++) {
            Pose pose = poses[i];
            drawPose(pose, color);
            if (i > 0) {
                Pose prevPose = poses[i - 1];
                canvas.strokeLine(prevPose.x, prevPose.y, pose.x, pose.y);
            }
        }
    }

    public void drawPose(Pose pose, String color) {
        double hInRadians = Math.toRadians(pose.h);
        canvas
                .setStroke(color)
                .setRotation(hInRadians)
                .strokeCircle(pose.x, pose.y, POSE_RADIUS)
                .strokeLine(pose.x, pose.y, pose.x + (POSE_RADIUS * Math.cos(hInRadians)), pose.y + (POSE_RADIUS * Math.sin(hInRadians)));
    }

    /**
     * @param currentPose whether to enable currentPose drawing (true by default)
     * @param targetPose whether to enable targetPose drawing (true by default)
     * @param path whether to enable path drawing (false by default)
     */
    public void enable(boolean currentPose, boolean targetPose, boolean path) {
        enableCurrentPose = currentPose;
        enableTargetPose = targetPose;
        enablePath = path;
    }

    public void setPath(Path path) { this.path = path; }
}
