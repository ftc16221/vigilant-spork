package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;

public class ColorThreshold {

    public float[] min = new float[3];
    public float[] max = new float[3];

    public ColorThreshold(float minHue, float maxHue, float minSat, float maxSat, float minVal, float maxVal) {
        min[0] = minHue;
        max[0] = maxHue;
        min[1] = minSat;
        max[1] = maxSat;
        min[2] = minVal;
        max[2] = maxVal;
    }

    public ColorThreshold(float hue, float sat, float val, float hueTolerance, float satValTolerance) {
        min[0] = addHue(hue, -hueTolerance);
        max[0] = addHue(hue, hueTolerance);
        min[1] = addValOrSat(sat, -satValTolerance);
        max[1] = addValOrSat(sat, satValTolerance);
        min[2] = addValOrSat(val, -satValTolerance);
        max[2] = addValOrSat(val, satValTolerance);
    }

    public ColorThreshold(int color1, int color2) {
        float[] hsv1 = new float[3];
        float[] hsv2 = new float[3];
        Color.colorToHSV(color1, hsv1);
        Color.colorToHSV(color2, hsv2);
        for (int i = 0; i < 3; i++) {
            if (hsv1[i] < hsv2[i]) {
                min[i] = hsv1[i];
                max[i] = hsv2[i];
            } else {
                min[i] = hsv2[i];
                max[i] = hsv1[i];
            }
        }
    }

    public boolean isValid() {
        boolean result = true;
        for (int i = 0; i < 3; i++) {
            if (min[i] > max[i]) {
                result = false;
                break;
            }
        }
        return result;
    }

    public boolean isColorInRange(float hue, float sat, float val) {
        if(!isValid()) {
            return false;
        }
        boolean result = true;
        if (hue < min[0] || hue > max[0]) result = false;
        if (val < min[1] || val > max[1]) result = false;
        if (sat < min[2] || sat > max[2]) result = false;
        return result;
    }

    public boolean isColorInRange(int color) {
        float[] hsv = new float[3];
        Color.colorToHSV(color, hsv);
        return isColorInRange(hsv[0], hsv[1], hsv[2]);
    }

    private float addHue(float h1, float h2) {
        float newHue = h1 + h2;
        while (newHue > 300f) {
            newHue -= 300f;
        }
        while (newHue < 0f) {
            newHue += 300f;
        }
        return newHue;
    }

    private float addValOrSat(float v1, float v2) {
        float v = v1 + v2;
        return (float) MathEx.clamp(v, 0f, 1f);
    }
}
