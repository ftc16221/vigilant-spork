package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;

import java.util.ArrayList;

public class VuMarkDetector implements Runnable {
    private final LinearOpMode opMode;
    private final VuforiaCurrentGame currentGame;

    public interface Listener {
        void handle(VuforiaBase.TrackingResults trackingResults);
    }
    private static ArrayList<Listener> listeners = new ArrayList<>();
    public static void addListener(Listener listener) {
        listeners.add(listener);
    }
    public static void clearListeners() {
        listeners.clear();
    }
    private static void dispatchListeners(VuforiaBase.TrackingResults trackingResults) {
        for(Listener listener : listeners) {
            listener.handle(trackingResults);
        }
    }
    public static void startThread(VuforiaCurrentGame currentGame, LinearOpMode opMode) {
        new Thread(new VuMarkDetector(currentGame, opMode)).start();
    }

    public VuMarkDetector(VuforiaCurrentGame currentGame, LinearOpMode opMode) {
        this.currentGame = currentGame;
        this.opMode = opMode;
        this.currentGame.activate();
    }

    @Override
    public void run() {
        // watch for vuMarks - when detected, shout.
        VuforiaBase.TrackingResults vuMarkResult;
        while (opMode.opModeIsActive()) {
            for (String trackable :
                    VuforiaCurrentGame.TRACKABLE_NAMES) {
                vuMarkResult = currentGame.track(trackable);
                if (vuMarkResult.isVisible) {
                    // vuMark ahoy!
                    dispatchListeners(vuMarkResult);
                }
                opMode.idle();
            }
        }

    }
}
