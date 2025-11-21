package org.firstinspires.ftc.teamcode.util;

public class CircularDoubleArray {
    private final double[] values;
    private int index = 0;
    private int count = 0;
    private double sum = 0;

    public CircularDoubleArray(int size) {
        values = new double[size];
    }

    public void addValue(double newValue) {
        // Subtract the old value being replaced
        sum -= values[index];
        // Add the new value
        values[index] = newValue;
        sum += newValue;

        // Move to the next index (circular)
        index = (index + 1) % values.length;
        if (count < values.length) count++;
    }

    public double getAverage() {
        if (count == 0) return 0;
        return sum / count;
    }

    public void reset() {
        for (int i = 0; i < values.length; i++) {
            values[i] = 0;
        }
        sum = 0;
        index = 0;
        count = 0;
    }
}
