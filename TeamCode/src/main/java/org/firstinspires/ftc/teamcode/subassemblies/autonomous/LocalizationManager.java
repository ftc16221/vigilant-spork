package org.firstinspires.ftc.teamcode.subassemblies.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subassemblies.Watchdog;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.LimelightCam;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import javax.annotation.CheckForNull;

public class LocalizationManager extends Subassembly {

    public static double LINEAR_SPEED_TOLERANCE = 3.0;
    public static double ANGULAR_SPEED_TOLERANCE = 2.0;

    private ArrayList<Localizer> localizers = null;
    private final ArrayList<Localizer> absoluteLocalizers = new ArrayList<>();
    private final ArrayList<Localizer> relativeLocalizers = new ArrayList<>();

    LimelightCam limelightCam = null;
    boolean useMT2 = false; // MT2 = MegaTag2

    private Pose pose;
    private Pose velocity;
    private double time;

    public LocalizationManager(OpMode opMode, Localizer... localizers) {
        super(opMode, "Localizer Manager");
        this.localizers = new ArrayList<>(Arrays.asList(localizers));

        for (Localizer localizer : localizers) {
            if (localizer.isAbsolute) absoluteLocalizers.add(localizer);
            else relativeLocalizers.add(localizer);

            if (localizer instanceof LimelightCam) {
                limelightCam = (LimelightCam) localizer;
            }
            localizer.update();
        }

        if (limelightCam != null) {
            useMT2 = !relativeLocalizers.isEmpty();
            limelightCam.useMT2 = useMT2;
        }

        Watchdog.i("LocalizerManager successfully initialized with the following Localizers: " + Arrays.toString(localizers));
    }

    public void start() {
//        sensorThread.start();
    }

    public void update() {

        double prevTime = time;
        time = System.nanoTime() / 1e9;
        double dt = time - prevTime;

        localizers.forEach(Localizer::update);

        Localizer mostAccurateLocalizer = getMostAccurateLocalizer(absoluteLocalizers);
        if (mostAccurateLocalizer == null) {
            mostAccurateLocalizer = getMostAccurateLocalizer(relativeLocalizers);
            if (mostAccurateLocalizer == null) {
                Watchdog.e("No valid localizers detected!");
                pose = null;
                velocity = null;
                return;
            }
        }
        Pose prevPose = pose;
        pose = mostAccurateLocalizer.getPose();

        // if we're using the limelight & MT2, we need to feed it heading data from other sensors, and make sure to never set heading data based off of it's output
        if (useMT2 && mostAccurateLocalizer instanceof LimelightCam) {
            Localizer mostAccurateRelativeLocalizer = getMostAccurateLocalizer(relativeLocalizers);
            pose.h = mostAccurateRelativeLocalizer.getPose().h;
            limelightCam.updateOrientation(pose.h);
        }

        // this might not be necessary all the time, and could be slowing loop times?
//            if (mostAccurateLocalizer.isAbsolute) {
//                relativeLocalizers.forEach(localizer -> localizer.setPose(pose));
//            }

        if (prevPose != null) {
            velocity = pose.subtract(prevPose).divideBy(dt);
        }

    }

    public void stop() {
    }

    private Localizer getMostAccurateLocalizer(List<Localizer> localizers) {
        Localizer mostAccurateLocalizer = null;

        List<Localizer> validLocalizers = localizers
                .stream()
                .filter(localizer -> localizer.getPose() != null)
                .collect(Collectors.toList());

        if (!validLocalizers.isEmpty()) {
            for (Localizer localizer : validLocalizers) {
                if (mostAccurateLocalizer == null || localizer.accuracy > mostAccurateLocalizer.accuracy) {
                    mostAccurateLocalizer = localizer;
                }
            }
        }
        return mostAccurateLocalizer;
    }

    @CheckForNull
    public Pose getPose() {
        return pose;
    }

    @CheckForNull
    public Pose getVelocity() {
        return velocity;
    }

    public double getLinearSpeed() {
        if (velocity == null) return 0.0;
        return Math.hypot(velocity.x, velocity.y);
    }

    public double getAngularSpeed() {
        if (velocity == null) return 0.0;
        return Math.abs(velocity.h);
    }

    public boolean isRobotMoving() {
        return getLinearSpeed() > LINEAR_SPEED_TOLERANCE || getAngularSpeed() > ANGULAR_SPEED_TOLERANCE;
    }
}
