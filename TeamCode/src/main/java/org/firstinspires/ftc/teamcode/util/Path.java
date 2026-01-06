package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subassemblies.autonomous.Navigator;


public class Path {

    public int state = 0;
    private int prevState = -1;
    private Pose[] value;
    private boolean isComplete = false;

    public Path(Pose... poses) {
        value = poses;
    }

    public boolean execute(Navigator navigator) {
        if (state != prevState) {
            if (isComplete) return true;
            if (state >= value.length) {
                isComplete = true;
                RobotLog.i("Path Complete!");
                return true;
            } else if (state + 1 == value.length) { // set ControllerType to APPROACH when approaching the last pose, otherwise default to DRIVE
                navigator.setControllerType(Navigator.ControllerType.APPROACH);
            } else {
                navigator.setControllerType(Navigator.ControllerType.DRIVE);
            }
            navigator.setTargetPose(value[state]);
            prevState = state;
            return false;
        }
        if (navigator.isAtTarget()) state++;
        return false;
    }

    /** resets the state to the initial value so that the Path can be executed again */
    public void reinitialize() {
        prevState = -1;
        state = 0;
        isComplete = false;
    }

    public void set(Pose[] value) { this.value = value; }
    public Pose[] get() { return value; }
    public boolean isComplete() { return isComplete; }

}
