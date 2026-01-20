package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class ToggleServo {

    private final Servo servo;

    public ToggleServo(Servo servo) {
        this.servo = servo;
    }

    public void toggle() {
        if (isOpen()) close();
        else open();
    }

    public void open() { servo.setPosition(1.0); }
    public void close() { servo.setPosition(0.0); }
    public boolean isOpen() { return servo.getPosition() > 0.5; }

    public void setScaleRange(double min, double max) { servo.scaleRange(min, max); }

    public void setDirection(Servo.Direction direction) { servo.setDirection(direction); };
    public Servo.Direction getDirection() { return servo.getDirection(); }
}
