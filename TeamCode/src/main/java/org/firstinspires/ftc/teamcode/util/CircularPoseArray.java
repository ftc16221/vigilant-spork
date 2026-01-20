package org.firstinspires.ftc.teamcode.util;

public class CircularPoseArray {

    Pose[] values;
    private int index = 0;
    private int count = 0;
    Pose sum = new Pose(0, 0, 0);

    public CircularPoseArray(int size) {
        values = new Pose[size];
    }

    public void add(Pose newValue) {
        if (values[index] != null) {
            // Subtract the old value being replaced
            sum = sum.subtract(values[index]);
        }
        // Add the new value
        values[index] = newValue;
        sum = sum.add(newValue);

        // Move to the next index (circular)
        index = (index + 1) % values.length;
        if (count < values.length) count++;
    }

    public Pose getAverage() {
        if (count == 0) return new Pose(0, 0, 0);
        return sum.divideBy(count);
    }

    public void reset() {
        for (int i = 0; i < values.length; i++) {
            values[i] = new Pose(0, 0, 0);
        }
        sum = new Pose(0, 0, 0);
        index = 0;
        count = 0;
    }

    public Pose get(int i) { return values[i]; }
}