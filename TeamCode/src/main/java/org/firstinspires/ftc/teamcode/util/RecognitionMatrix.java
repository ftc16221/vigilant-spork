package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class RecognitionMatrix {
    public Recognition recognition;
    public OpenGLMatrix position;
    public RecognitionMatrix(Recognition recognition) {
        this.recognition = recognition;
        float avgX = (recognition.getLeft() + recognition.getRight()) / 2;
        float avgY = (recognition.getTop() + recognition.getBottom()) / 2;
        this.position = OpenGLMatrix.translation(avgX, avgY, 0); // 2D matrix?
    }
}
